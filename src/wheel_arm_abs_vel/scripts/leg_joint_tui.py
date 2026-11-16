#!/usr/bin/env python3
"""Beta lower-body TUI layout with the current full-body IK service.

The original joint1..joint4 fields, degree ranges, Publish and Zero buttons
are retained. Zero fills all four targets with zero and submits once.
Degrees in UI; radians in the service.

The IK owns interpolation, limits and execution authority. No /lb_leg_traj
publisher and no legacy lock-service fallback are used here.
"""
import asyncio
import math
import queue
import threading
import time

import rospy
from wheel_arm_abs_vel.srv import ManualLowerBody, ManualLowerBodyRequest as Request
from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Button, Footer, Header, Input, Label

SERVICE = "/whole_body_velocity_ik/manual_lower_body"
JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4")
JOINT_MIN = (0., -60., 0., 0.)
JOINT_MAX = (60., 0., 0., 0.)
STATES = ("空闲", "等待执行", "调整中", "到位并保持", "正在停止", "已中止并保持")


def parse_target(values, lower, upper):
    target = [math.radians(float(value)) for value in values]
    if len(target) != 4 or any(not math.isfinite(x) or x < lo or x > hi
                             for x, lo, hi in zip(target, lower, upper)):
        raise ValueError("目标必须是限位范围内的四个有限角度")
    return target


class LegJointTui(App):
    TITLE = "下肢关节控制"
    BINDINGS = [("q", "quit", "退出"), ("escape", "stop", "停止调整")]
    CSS = """
    .joint-row { height: 3; }
    .joint-name { width: 10; content-align: right middle; padding-right: 1; }
    .joint-input { width: 12; }
    .joint-range { width: 16; color: $text-muted; padding-left: 1; }
    .joint-current { width: 16; color: $text-success; padding-left: 1; }
    #status { color: $text-muted; height: 2; }
    #buttons { height: 3; }
    """

    def compose(self) -> ComposeResult:
        yield Header()
        for name, lo, hi in zip(JOINT_NAMES, JOINT_MIN, JOINT_MAX):
            with Horizontal(classes="joint-row"):
                yield Label(name, classes="joint-name")
                yield Input(value="0.0", id=f"in_{name}", classes="joint-input")
                yield Label(f"{lo:g} ~ {hi:g}", classes="joint-range")
                yield Label("当前: --", id=f"cur_{name}", classes="joint-current")
        yield Label("等待全身 IK 服务…", id="status")
        with Horizontal(id="buttons"):
            yield Button("发布", id="publish", variant="primary")
            yield Button("归零", id="zero")
        yield Footer()

    def on_mount(self):
        self._commands = queue.Queue()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = None
        self._received = 0.
        self._error = ""
        self._pending = False
        self._owned_id = 0
        rospy.init_node("leg_joint_tui", anonymous=True, disable_signals=True)
        self._worker = threading.Thread(target=self._ros_worker, daemon=True)
        self._worker.start()
        self.set_interval(.2, self._refresh)

    def _ros_worker(self):
        proxy = rospy.ServiceProxy(SERVICE, ManualLowerBody)
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                rospy.wait_for_service(SERVICE, timeout=.5)
                try:
                    command, values = self._commands.get_nowait()
                except queue.Empty:
                    command, values = Request.QUERY, None
                req = Request(command=command)
                if command == Request.SET_TARGET:
                    req.position = values
                elif command == Request.CANCEL:
                    req.request_id = self._owned_id
                res = proxy(req)
                if command == Request.SET_TARGET and res.accepted:
                    self._owned_id = res.request_id
                with self._lock:
                    self._snapshot = res
                    self._received = time.monotonic()
                    if command != Request.QUERY:
                        self._pending = False
                        self._error = "" if res.accepted else "请求被拒绝: " + res.message
            except (rospy.ROSException, rospy.ServiceException) as exc:
                # Never retry a SET after an ambiguous transport failure.
                with self._lock:
                    self._pending = False
                    self._received = 0.
                    self._error = "全身 IK 服务不可用，请检查节点及工作空间: " + str(exc)
            self._stop.wait(.2)
        # Best effort on normal exit; the server also has an independent timeout.
        if self._owned_id:
            try:
                proxy(Request(command=Request.CANCEL, request_id=self._owned_id))
            except (rospy.ROSException, rospy.ServiceException):
                pass

    @staticmethod
    def _reason(message):
        return {
            "Pause arms with X+B before adjusting": "请先按 X+B 暂停手臂",
            "Lock torso following before adjusting": "请先锁定整个下肢（X＋右摇杆下拉）",
            "Enter healthy full-body following with fresh VR and robot feedback first":
                "等待有效的 VR 输入、机器人反馈和 IK 接管",
            "Wait for standstill and a fresh IK snapshot": "等待机器人停稳及 IK 状态更新",
            "Wait for arms and lower body to stop": "等待手臂和下肢停稳",
            "VR session or operator mode changed": "会话或操作模式已变化，本次调整已中止",
            "Cancelled by TUI": "已取消调整",
        }.get(message, message)

    def _refresh(self):
        with self._lock:
            res, received, error, pending = self._snapshot, self._received, self._error, self._pending
        fresh = res is not None and time.monotonic() - received < 1.
        if not fresh:
            text = error or "等待全身 IK 服务…"
        else:
            for j, name in enumerate(JOINT_NAMES):
                self.query_one(f"#cur_{name}", Label).update(
                    f"当前: {math.degrees(res.measured_position[j]):.1f}°")
            if error:
                text = error
            elif pending:
                text = "正在提交…"
            elif res.state in (1, 2, 4):
                text = STATES[res.state]
            elif res.state == 3:
                text = "已到位并保持，按 X+B 恢复手臂遥操"
            elif res.ready:
                text = "就绪，等待发布"
            else:
                text = "暂不可发布：" + self._reason(res.message)
        self.query_one("#status", Label).update(text)

    def _publish(self):
        with self._lock:
            res = self._snapshot
            if self._pending:
                self._error = "正在提交，请稍候"
            elif not res or time.monotonic()-self._received >= 1.:
                self._error = "全身 IK 服务未就绪"
            elif not res.ready:
                self._error = ("正在调整，请等待到位，或按 Esc 停止"
                               if res.state in (1, 2, 4) else self._reason(res.message))
            else:
                try:
                    values = []
                    for name, lo, hi in zip(JOINT_NAMES, JOINT_MIN, JOINT_MAX):
                        field = self.query_one(f"#in_{name}", Input)
                        value = float(field.value)
                        if not math.isfinite(value):
                            raise ValueError("输入非法，请检查各关节数值")
                        clamped = max(lo, min(hi, value))
                        if clamped != value:
                            field.value = f"{clamped:g}"
                        values.append(str(clamped))
                    goal = parse_target(values, res.lower, res.upper)
                except ValueError:
                    self._error = "输入非法或超出机器人限位，请检查各关节数值"
                else:
                    self._pending = True
                    self._error = ""
                    self._commands.put((Request.SET_TARGET, goal))
        self._refresh()

    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id == "publish":
            self._publish()
        elif event.button.id == "zero":
            for name in JOINT_NAMES:
                self.query_one(f"#in_{name}", Input).value = "0.0"
            self._publish()

    def action_stop(self):
        if self._owned_id:
            self._commands.put((Request.CANCEL, None))

    async def action_quit(self):
        self._stop.set()
        await asyncio.get_running_loop().run_in_executor(None, self._worker.join, 1.)
        self.exit()

    def on_unmount(self):
        self._stop.set()


if __name__ == "__main__":
    LegJointTui().run(mouse=True)
