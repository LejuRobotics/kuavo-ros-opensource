#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys

os.environ["QT_QPA_PLATFORM"] = os.environ.get("QT_QPA_PLATFORM", "xcb")
os.environ["QT_FONT_DPI"] = "96"

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import threading

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QSizePolicy, QFrame, QSlider, QGraphicsDropShadowEffect
)
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPainter, QColor, QPen, QBrush, QLinearGradient

# Xbox style button and axis constants
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7
BUTTON_LOGITECH = 8
BUTTON_LS = 9
BUTTON_RS = 10

AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4


class JoystickCircle(QWidget):
    """
    Beautiful circular joystick widget, supports mouse drag, outputs x, y in [-1, 1]
    """
    valueChanged = pyqtSignal(float, float)  # x, y

    def __init__(self, color=QColor(80, 170, 255), parent=None):
        super().__init__(parent)
        self.radius = 70
        self.knob_radius = 22
        self.center = QPointF(self.radius + 16, self.radius + 16)
        self.knob_pos = QPointF(self.center)
        self.dragging = False
        self.x = 0.0
        self.y = 0.0
        self.color = color
        self.setFixedSize(self.radius * 2 + 32, self.radius * 2 + 32)
        self.setStyleSheet("background: transparent;")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        # Draw shadow
        shadow_color = QColor(0, 0, 0, 60)
        painter.setBrush(QBrush(shadow_color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(self.center + QPointF(4, 6), self.radius, self.radius)
        # Draw main circle with gradient
        grad = QLinearGradient(float(self.center.x() - self.radius),
                               float(self.center.y() - self.radius),
                               float(self.center.x() + self.radius),
                               float(self.center.y() + self.radius))
        grad.setColorAt(0, QColor(240, 245, 255))
        grad.setColorAt(1, QColor(210, 220, 240))
        painter.setBrush(QBrush(grad))
        painter.setPen(QPen(QColor(120, 160, 220), 3))
        painter.drawEllipse(self.center, self.radius, self.radius)
        # Draw cross lines
        painter.setPen(QPen(QColor(180, 200, 230), 1, Qt.DashLine))
        painter.drawLine(int(self.center.x() - self.radius),
                         int(self.center.y()),
                         int(self.center.x() + self.radius),
                         int(self.center.y()))
        painter.drawLine(int(self.center.x()),
                         int(self.center.y() - self.radius),
                         int(self.center.x()),
                         int(self.center.y() + self.radius))
        # Draw knob with highlight
        knob_grad = QLinearGradient(
            float(self.knob_pos.x() - self.knob_radius),
            float(self.knob_pos.y() - self.knob_radius),
            float(self.knob_pos.x() + self.knob_radius),
            float(self.knob_pos.y() + self.knob_radius))
        knob_grad.setColorAt(0, QColor(255, 255, 255))
        knob_grad.setColorAt(1, self.color)
        painter.setPen(QPen(QColor(60, 120, 200), 2))
        painter.setBrush(QBrush(knob_grad))
        painter.drawEllipse(self.knob_pos, self.knob_radius, self.knob_radius)
        # Draw border
        painter.setPen(QPen(QColor(80, 120, 180), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(self.center, self.radius, self.radius)

    def mousePressEvent(self, event):
        if (event.pos() - self.knob_pos).manhattanLength() < self.knob_radius + 8:
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            dx = event.x() - self.center.x()
            dy = event.y() - self.center.y()
            dist = (dx ** 2 + dy ** 2) ** 0.5
            if dist > self.radius:
                dx = dx * self.radius / dist
                dy = dy * self.radius / dist
            self.knob_pos = QPointF(self.center.x() + dx, self.center.y() + dy)
            self.x = -dx / self.radius
            self.y = -dy / self.radius
            self.valueChanged.emit(self.x, self.y)
            self.update()

    def mouseReleaseEvent(self, event):
        if self.dragging:
            self.dragging = False

    def mouseDoubleClickEvent(self, event):
        self.reset()

    def reset(self):
        self.knob_pos = QPointF(self.center)
        self.x = 0.0
        self.y = 0.0
        self.valueChanged.emit(self.x, self.y)
        self.update()

    def setXY(self, x, y):
        # x, y in [-1, 1]
        dx = -x * self.radius
        dy = -y * self.radius
        self.knob_pos = QPointF(self.center.x() + dx, self.center.y() + dy)
        self.x = x
        self.y = y
        self.update()


class XYawJoystickCircle(JoystickCircle):
    """
    Circular joystick with toggled dragging: single click to start following mouse,
    single click again to cancel following mouse.
    """

    def __init__(self, color=QColor(80, 170, 255), parent=None):
        super().__init__(color, parent)
        self.follow_mode = False
        self.setMouseTracking(True)  # 确保moveEvent全局响应

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.follow_mode = not self.follow_mode

    def handle_absolute_pos(self, global_pos):
        # 接收全局坐标，映射到stick中心坐标系
        stick_global_center = self.mapToGlobal(self.center.toPoint())
        dx = global_pos.x() - stick_global_center.x()
        dy = global_pos.y() - stick_global_center.y()
        if self.radius > 0:
            norm_x = -dx / self.radius
            norm_y = -dy / self.radius
            norm_x = max(min(norm_x, 1.0), -1.0)
            norm_y = max(min(norm_y, 1.0), -1.0)
            dx_display = -norm_x * self.radius
            dy_display = -norm_y * self.radius
            self.knob_pos = QPointF(self.center.x() + dx_display, self.center.y() + dy_display)
            self.x = norm_x
            self.y = norm_y
            self.valueChanged.emit(self.x, self.y)
            self.update()
        else:
            self.knob_pos = QPointF(self.center)
            self.x = 0.0
            self.y = 0.0
            self.valueChanged.emit(self.x, self.y)
            self.update()

    def mouseMoveEvent(self, event):
        # 此处不会收到label之上的事件，用窗口级同步
        if self.follow_mode:
            self.handle_absolute_pos(self.mapToGlobal(event.pos()))

    def mouseReleaseEvent(self, event):
        pass

    def mouseDoubleClickEvent(self, event):
        self.reset()
        self.follow_mode = False

    def reset(self):
        super().reset()
        self.follow_mode = False


class SimpleButton(QPushButton):
    """
    Beautiful gamepad button
    """
    def __init__(self, label, idx, joy_msg, color, callback=None, parent=None):
        super().__init__(label, parent)
        self.idx = idx
        self.joy_msg = joy_msg
        self.callback = callback
        self.setCheckable(True)
        self.setFixedSize(48, 48)
        self.setFont(QFont("Segoe UI", 14, QFont.Bold))
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border-radius: 24px;
                border: 2px solid #b0c4de;
            }}
            QPushButton:pressed {{
                background-color: #222;
                color: #fff;
                border: 2px solid #222;
            }}
        """)
        self.pressed.connect(self.on_press)
        self.released.connect(self.on_release)

    def on_press(self):
        self.joy_msg.buttons[self.idx] = 1
        if self.callback:
            self.callback()

    def on_release(self):
        self.joy_msg.buttons[self.idx] = 0
        if self.callback:
            self.callback()

class CmdVelBridge(QObject):
    # 信号用于主线程安全地更新UI
    update_cmd_vel = pyqtSignal(float, float, float)

    def __init__(self, ui):
        super().__init__()
        self.ui = ui
        self.update_cmd_vel.connect(self._do_update)

    def _do_update(self, vx, vy, yaw):
        self.ui.set_from_cmd_vel(vx, vy, yaw)


class XYawStickWindow(QWidget):
    """
    单独的小窗口，显示x-yaw圆盘
    """    
    def __init__(self, main_ui):
        super().__init__()
        self.setWindowTitle("X-Yaw Joystick")
        self.setFixedSize(260, 260)
        self.setStyleSheet("""
            QWidget {
                background: #eaf2fb;
                border-radius: 18px;
            }
        """)
        # 让窗口始终置顶
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.main_ui = main_ui
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        label = QLabel("X-Yaw Joystick")
        label.setFont(QFont("Segoe UI", 14, QFont.Bold))
        label.setStyleSheet("color: #3a5fa0; background: none;")
        label.setAttribute(Qt.WA_TransparentForMouseEvents, True)  # 关键：穿透label的鼠标事件
        layout.addWidget(label, alignment=Qt.AlignHCenter)

        # 使用特别定制的 XYawJoystickCircle
        self.xyaw_stick = XYawJoystickCircle(QColor(120, 200, 120))
        self.xyaw_stick.setMouseTracking(True)
        layout.addWidget(self.xyaw_stick, alignment=Qt.AlignCenter)
        self.setLayout(layout)
        self.xyaw_stick.valueChanged.connect(self.on_xyaw_stick_changed)

        self.setMouseTracking(True)

    def mouseMoveEvent(self, event):
        # 任何区域(包括label)的鼠标移动均采集
        child = self.xyaw_stick
        global_pos = self.mapToGlobal(event.pos())
        if child.follow_mode:
            child.handle_absolute_pos(global_pos)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        # 任何区域单击均可切换跟随模式
        child = self.xyaw_stick
        if event.button() == Qt.LeftButton:
            # 构造pos为stick中心的鼠标事件，保证哪怕点击label也正确切换
            from PyQt5.QtGui import QMouseEvent
            fake_pos = child.center
            fake_ev = QMouseEvent(event.type(), fake_pos, event.globalPos(), event.button(), event.buttons(), event.modifiers())
            QApplication.sendEvent(child, fake_ev)
        super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        child = self.xyaw_stick
        from PyQt5.QtGui import QMouseEvent
        fake_pos = child.center
        fake_ev = QMouseEvent(event.type(), fake_pos, event.globalPos(), event.button(), event.buttons(), event.modifiers())
        QApplication.sendEvent(child, fake_ev)
        super().mouseDoubleClickEvent(event)

    def on_xyaw_stick_changed(self, x, y):
        self.main_ui.on_xyaw_stick_changed(x, y)

    def setXY(self, x, y):
        self.xyaw_stick.setXY(x, y)


class JoystickUI(QWidget):
    def __init__(self, joy_msg, joy_pub):
        super().__init__()
        self.joy_msg = joy_msg
        self.joy_pub = joy_pub
        self.setWindowTitle("Virtual Joystick Controller")
        self.setFixedSize(900, 520)
        font = QFont("Segoe UI", 12)
        self.setFont(font)
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #eaf2fb, stop:1 #c9d6e8);
                border-radius: 18px;
            }
        """)
        # # 让主窗口始终置顶
        # self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_joy)
        self.timer.start(50)  # 20Hz
        self._block_slider_update = False
        self._block_stick_update = False
        self._block_xyaw_update = False  # 新增：防止递归
        self.xyaw_window = None  # 新增：x-yaw圆盘窗口
        self.initUI()

    def initUI(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(32, 24, 32, 24)
        main_layout.setSpacing(18)

        # Title
        title = QLabel("Virtual Joystick")
        title.setFont(QFont("Segoe UI", 22, QFont.Bold))
        title.setStyleSheet("color: #3a5fa0; margin-bottom: 8px; background: none;")
        main_layout.addWidget(title, alignment=Qt.AlignHCenter)

        # Joysticks and sliders
        center_layout = QHBoxLayout()
        center_layout.setSpacing(40)

        # Left stick
        self.left_stick = JoystickCircle(QColor(80, 170, 255))
        self.left_stick.valueChanged.connect(self.on_left_stick_changed)

        # Sliders
        sliders_frame = QFrame()
        sliders_layout = QVBoxLayout()
        sliders_layout.setSpacing(18)
        sliders_layout.setContentsMargins(10, 10, 10, 10)

        # vx slider
        vx_label = QLabel("vx")
        vx_label.setFont(QFont("Segoe UI", 13, QFont.Bold))
        self.vx_slider = QSlider(Qt.Horizontal)
        self.vx_slider.setMinimum(-100)
        self.vx_slider.setMaximum(100)
        self.vx_slider.setValue(0)
        self.vx_slider.setSingleStep(1)
        self.vx_slider.setTickInterval(20)
        self.vx_slider.setTickPosition(QSlider.TicksBelow)
        self.vx_slider.valueChanged.connect(self.on_vx_slider_changed)

        # vy slider
        vy_label = QLabel("vy")
        vy_label.setFont(QFont("Segoe UI", 13, QFont.Bold))
        self.vy_slider = QSlider(Qt.Horizontal)
        self.vy_slider.setMinimum(-100)
        self.vy_slider.setMaximum(100)
        self.vy_slider.setValue(0)
        self.vy_slider.setSingleStep(1)
        self.vy_slider.setTickInterval(20)
        self.vy_slider.setTickPosition(QSlider.TicksBelow)
        self.vy_slider.valueChanged.connect(self.on_vy_slider_changed)

        # yaw slider
        yaw_label = QLabel("yaw")
        yaw_label.setFont(QFont("Segoe UI", 13, QFont.Bold))
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setMinimum(-100)
        self.yaw_slider.setMaximum(100)
        self.yaw_slider.setValue(0)
        self.yaw_slider.setSingleStep(1)
        self.yaw_slider.setTickInterval(20)
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.valueChanged.connect(self.on_yaw_slider_changed)

        sliders_layout.addWidget(vx_label)
        sliders_layout.addWidget(self.vx_slider)
        sliders_layout.addWidget(vy_label)
        sliders_layout.addWidget(self.vy_slider)
        sliders_layout.addWidget(yaw_label)
        sliders_layout.addWidget(self.yaw_slider)
        sliders_frame.setLayout(sliders_layout)

        # Right stick
        self.right_stick = JoystickCircle(QColor(255, 170, 80))
        self.right_stick.valueChanged.connect(self.on_right_stick_changed)

        # center_layout: 左圆盘 | 滑块 | 右圆盘
        center_layout.addWidget(self.left_stick, alignment=Qt.AlignLeft)
        center_layout.addWidget(sliders_frame, alignment=Qt.AlignVCenter)
        center_layout.addWidget(self.right_stick, alignment=Qt.AlignRight)
        main_layout.addLayout(center_layout)

        # Buttons
        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(18)
        btns = [
            ("A", BUTTON_A, "#4caf50"),
            ("B", BUTTON_B, "#e53935"),
            ("X", BUTTON_X, "#1e88e5"),
            ("Y", BUTTON_Y, "#fbc02d"),
            ("LB", BUTTON_LB, "#7e57c2"),
            ("RB", BUTTON_RB, "#26a69a"),
            ("START", BUTTON_START, "#757575")
        ]
        self.button_widgets = []
        for label, idx, color in btns:
            btn = SimpleButton(label, idx, self.joy_msg, color, self.update_status)
            self.button_widgets.append(btn)
            buttons_layout.addWidget(btn)
        main_layout.addLayout(buttons_layout)

        # Status Frame
        status_frame = QFrame()
        status_frame.setStyleSheet("""
            QFrame {
                background: #f7fafc;
                border-radius: 12px;
                border: 1.5px solid #b0c4de;
                padding: 10px 18px;
            }
        """)
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setSpacing(16)
        self.status_label = QLabel()
        self.status_label.setFont(QFont("Consolas", 13, QFont.Bold))
        self.status_label.setStyleSheet("color: #2d3e50;")
        status_layout.addWidget(self.status_label, alignment=Qt.AlignCenter)
        status_frame.setLayout(status_layout)
        main_layout.addWidget(status_frame, alignment=Qt.AlignHCenter)
        self.setLayout(main_layout)
        self.update_status()

        # 新增：x-yaw圆盘窗口（用自动归零定制版）
        self.xyaw_window = XYawStickWindow(self)
        self.xyaw_window.show()

    def on_left_stick_changed(self, x, y):
        if self._block_stick_update:
            return
        self._block_slider_update = True
        self._block_xyaw_update = True
        # 更新joy_msg
        self.joy_msg.axes[AXIS_LEFT_STICK_X] = x
        self.joy_msg.axes[AXIS_LEFT_STICK_Y] = y
        # 同步滑块
        self.vy_slider.setValue(int(x * 100))
        self.vx_slider.setValue(int(y * 100))
        # 同步x-yaw圆盘
        self.xyaw_window.setXY(self.joy_msg.axes[AXIS_RIGHT_STICK_X], y)
        self._block_slider_update = False
        self._block_xyaw_update = False
        self.update_status()

    def on_right_stick_changed(self, x, y):
        if self._block_stick_update:
            return
        self._block_slider_update = True
        self._block_xyaw_update = True
        self.joy_msg.axes[AXIS_RIGHT_STICK_X] = x
        self.joy_msg.axes[AXIS_RIGHT_STICK_Y] = y
        self.yaw_slider.setValue(int(x * 100))
        # 同步x-yaw圆盘
        self.xyaw_window.setXY(x, self.joy_msg.axes[AXIS_LEFT_STICK_Y])
        self._block_slider_update = False
        self._block_xyaw_update = False
        self.update_status()

    def on_xyaw_stick_changed(self, x, y):
        if self._block_xyaw_update:
            return
        self._block_stick_update = True
        self._block_slider_update = True
        # x为横向(yaw)，y为竖向(x速度)
        # 约定：x轴为yaw，y轴为x速度
        self.joy_msg.axes[AXIS_LEFT_STICK_Y] = y  # x速度
        self.joy_msg.axes[AXIS_RIGHT_STICK_X] = x  # yaw
        # 同步滑块
        self.vx_slider.setValue(int(y * 100))
        self.yaw_slider.setValue(int(x * 100))
        # 同步左圆盘
        self.left_stick.setXY(self.joy_msg.axes[AXIS_LEFT_STICK_X], y)
        # 同步右圆盘
        self.right_stick.setXY(x, 0)
        self._block_stick_update = False
        self._block_slider_update = False
        self.update_status()

    def on_vx_slider_changed(self, value):
        if self._block_slider_update:
            return
        self._block_stick_update = True
        self._block_xyaw_update = True
        y = value / 100.0
        self.joy_msg.axes[AXIS_LEFT_STICK_Y] = y
        # 同步圆盘
        self.left_stick.setXY(self.joy_msg.axes[AXIS_LEFT_STICK_X], y)
        # 同步x-yaw圆盘
        self.xyaw_window.setXY(self.joy_msg.axes[AXIS_RIGHT_STICK_X], y)
        self._block_stick_update = False
        self._block_xyaw_update = False
        self.update_status()

    def on_vy_slider_changed(self, value):
        if self._block_slider_update:
            return
        self._block_stick_update = True
        x = value / 100.0
        self.joy_msg.axes[AXIS_LEFT_STICK_X] = x
        self.left_stick.setXY(x, self.joy_msg.axes[AXIS_LEFT_STICK_Y])
        self._block_stick_update = False
        self.update_status()

    def on_yaw_slider_changed(self, value):
        if self._block_slider_update:
            return
        self._block_stick_update = True
        self._block_xyaw_update = True
        x = value / 100.0
        self.joy_msg.axes[AXIS_RIGHT_STICK_X] = x
        self.right_stick.setXY(x, self.joy_msg.axes[AXIS_RIGHT_STICK_Y])
        # 同步x-yaw圆盘
        self.xyaw_window.setXY(x, self.joy_msg.axes[AXIS_LEFT_STICK_Y])
        self._block_stick_update = False
        self._block_xyaw_update = False
        self.update_status()

    def update_status(self):
        vy = self.joy_msg.axes[AXIS_LEFT_STICK_X]
        vx = self.joy_msg.axes[AXIS_LEFT_STICK_Y]
        vyaw = self.joy_msg.axes[AXIS_RIGHT_STICK_X]
        self.status_label.setText(
            f"vx: <b>{vx:+.2f}</b>    vy: <b>{vy:+.2f}</b>    yaw: <b>{vyaw:+.2f}</b>"
        )

    def publish_joy(self):
        self.joy_pub.publish(self.joy_msg)
        # Release all buttons after publish (single frame)
        self.joy_msg.buttons = [0] * len(self.joy_msg.buttons)

    def set_from_cmd_vel(self, vx, vy, yaw):
        """
        vx, vy, yaw: float, normalized to [-1, 1]
        """
        self._block_stick_update = True
        self._block_slider_update = True
        self._block_xyaw_update = True

        # 更新joy_msg
        self.joy_msg.axes[AXIS_LEFT_STICK_X] = vy
        self.joy_msg.axes[AXIS_LEFT_STICK_Y] = vx
        self.joy_msg.axes[AXIS_RIGHT_STICK_X] = yaw

        # 更新UI控件
        self.left_stick.setXY(vy, vx)
        self.vy_slider.setValue(int(vy * 100))
        self.vx_slider.setValue(int(vx * 100))
        self.right_stick.setXY(yaw, 0.0)
        self.yaw_slider.setValue(int(yaw * 100))
        # 同步x-yaw圆盘
        self.xyaw_window.setXY(yaw, vx)

        self._block_stick_update = False
        self._block_slider_update = False
        self._block_xyaw_update = False
        self.update_status()

class SimulatedJoystickNode:
    def __init__(self):
        rospy.init_node('simulated_joystick')
        rospy.Subscriber("/stop_robot", Bool, self.stop_robot_callback)
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 11
        self.app = None
        self.ui = None
        self.shutdown_flag = False

        # 订阅cmd_vel
        self.cmd_vel_max_vx = rospy.get_param("~max_vx", 1.0)
        self.cmd_vel_max_vy = rospy.get_param("~max_vy", 1.0)
        self.cmd_vel_max_yaw = rospy.get_param("~max_yaw", 1.0)
        self.cmd_vel_bridge = None  # 用于主线程信号

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

    def stop_robot_callback(self, msg):
        self.shutdown_flag = True
        rospy.signal_shutdown("stop_robot")
        if self.app:
            self.app.quit()

    def run(self):
        self.app = QApplication(sys.argv)
        font = QFont("Segoe UI", 12)
        self.app.setFont(font)
        self.ui = JoystickUI(self.joy_msg, self.joy_pub)
        self.cmd_vel_bridge = CmdVelBridge(self.ui)
        self.ui.show()
        self.ui.xyaw_window.show()
        threading.Thread(target=self.spin_rospy, daemon=True).start()
        self.app.exec_()

    def spin_rospy(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.shutdown_flag:
            rate.sleep()

    def cmd_vel_callback(self, msg):
        # 将Twist消息的线速度和角速度映射到[-1, 1]，并更新UI
        vx = max(min(msg.linear.x / self.cmd_vel_max_vx, 1.0), -1.0) if self.cmd_vel_max_vx > 0 else 0.0
        vy = max(min(msg.linear.y / self.cmd_vel_max_vy, 1.0), -1.0) if self.cmd_vel_max_vy > 0 else 0.0
        yaw = max(min(msg.angular.z / self.cmd_vel_max_yaw, 1.0), -1.0) if self.cmd_vel_max_yaw > 0 else 0.0

        # 通过信号安全地在主线程更新UI
        if self.cmd_vel_bridge is not None:
            self.cmd_vel_bridge.update_cmd_vel.emit(vx, vy, yaw)

if __name__ == '__main__':
    try:
        node = SimulatedJoystickNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
