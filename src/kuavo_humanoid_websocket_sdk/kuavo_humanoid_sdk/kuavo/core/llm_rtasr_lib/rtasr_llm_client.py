# -*- encoding:utf-8 -*-
import hashlib
import hmac
import base64
import json
import time
import threading
import urllib.parse
import logging
import uuid
from websocket import create_connection, WebSocketException
import websocket
import datetime

# 全局配置：与服务端确认的固定参数
FIXED_PARAMS = {
    "audio_encode": "pcm_s16le",
    "lang": "autodialect",
    "samplerate": "16000",  # 固定16k采样率，对应每40ms发送1280字节
}
AUDIO_FRAME_SIZE = 1280  # 每帧音频字节数（16k采样率、16bit位深、40ms）
FRAME_INTERVAL_MS = 40  # 每帧发送间隔（毫秒）


class XFYunRTASRClient:
    def __init__(self, app_id, access_key_id, access_key_secret, base_url=None):
        self.app_id = app_id
        self.access_key_id = access_key_id
        self.access_key_secret = access_key_secret
        self.base_ws_url = (
            base_url
            if base_url
            else "wss://office-api-ast-dx.iflyaisol.com/ast/communicate/v1"
        )
        self.ws = None
        self.is_connected = False
        self.recv_thread = None
        self.session_id = None
        self.is_sending_audio = False  # 防止并发发送
        self.audio_file_size = 0  # 音频文件大小（字节）
        self.recognized_text = ""  # 存储识别出的文本

    def _generate_auth_params(self):
        """生成鉴权参数（严格按字典序排序，匹配Java TreeMap）"""
        auth_params = {
            "accessKeyId": self.access_key_id,
            "appId": self.app_id,
            "uuid": uuid.uuid4().hex,
            "utc": self._get_utc_time(),
            **FIXED_PARAMS,
        }

        # 计算签名：过滤空值 → 字典序排序 → URL编码 → 拼接基础字符串
        sorted_params = dict(
            sorted(
                [
                    (k, v)
                    for k, v in auth_params.items()
                    if v is not None and str(v).strip() != ""
                ]
            )
        )
        base_str = "&".join(
            [
                f"{urllib.parse.quote(k, safe='')}={urllib.parse.quote(v, safe='')}"
                for k, v in sorted_params.items()
            ]
        )

        # HMAC-SHA1 加密 + Base64编码
        signature = hmac.new(
            self.access_key_secret.encode("utf-8"),
            base_str.encode("utf-8"),
            hashlib.sha1,
        ).digest()
        auth_params["signature"] = base64.b64encode(signature).decode("utf-8")
        return auth_params

    def _get_utc_time(self):
        """生成服务端要求的UTC时间格式：yyyy-MM-dd'T'HH:mm:ss+0800"""
        beijing_tz = datetime.timezone(datetime.timedelta(hours=8))
        now = datetime.datetime.now(beijing_tz)
        return now.strftime("%Y-%m-%dT%H:%M:%S%z")

    def connect(self):
        """建立WebSocket连接（增加稳定性配置）"""
        try:
            auth_params = self._generate_auth_params()
            params_str = urllib.parse.urlencode(auth_params)
            full_ws_url = f"{self.base_ws_url}?{params_str}"
            print(f"【连接信息】完整URL：{full_ws_url}")

            # 初始化WebSocket连接（禁用自动文本解析，延长超时）
            self.ws = create_connection(
                full_ws_url, timeout=60, enable_multithread=True  # 支持多线程并发
            )
            self.is_connected = True
            print("【连接成功】WebSocket握手完成，等待服务端就绪...")
            time.sleep(1.0)  # 确保服务端完全初始化

            # 启动接收线程（单独处理服务端消息）
            # self.recv_thread = threading.Thread(target=self._recv_msg, daemon=True)
            # self.recv_thread.start()
            return True
        except WebSocketException as e:
            print(f"【连接失败】WebSocket错误：{str(e)}")
            if hasattr(e, "status_code"):
                print(f"【服务端状态码】{e.status_code}")
            return False
        except Exception as e:
            print(f"【连接异常】其他错误：{str(e)}")
            return False

    def _recv_msg(self):
        """接收服务端消息（同步实现，单次接收）"""
        # 先判断连接状态，避免操作已关闭的连接
        if not self.is_connected or not self.ws:
            print("【接收线程】连接已关闭，退出接收循环")
            return

        try:
            msg = self.ws.recv()
            if not msg:
                print("【接收消息】服务端关闭连接")
                self.close()
                return

            # 仅处理文本消息（服务端返回的JSON均为文本）
            if isinstance(msg, str):
                try:
                    msg_json = json.loads(msg)
                    print(f"【接收消息】{msg_json}")

                    # 更新会话ID（用于结束标记关联）
                    if msg_json.get(
                        "msg_type"
                    ) == "action" and "sessionId" in msg_json.get("data", {}):
                        self.session_id = msg_json["data"]["sessionId"]

                    # 解析识别结果（根据讯飞文档格式）
                    if msg_json.get("msg_type") == "result" and "data" in msg_json:
                        data = msg_json["data"]
                        # 检查是否为ASR识别结果
                        self.recognized_text = "".join(
                            [
                                text.get("cw", [{}])[0].get("w", "")
                                for text in msg_json.get("data", {})
                                .get("cn", {})
                                .get("st", {})
                                .get("rt", [{}])[0]
                                .get("ws", [{}])
                            ]
                        )

                    # 检查是否为最终结果，若是则关闭连接
                    print("最终结果:",(msg_json.get("data", {}).get("cn", {}).get("st", {}).get("type", None)))
                    if (
                        msg_json.get("data", {})
                        .get("cn", {})
                        .get("st", {})
                        .get("type", None)
                        == '0'
                    ):
                        print("【ASR结束】已收到最终识别结果")

                    return msg_json
                except json.JSONDecodeError:
                    print(f"【接收异常】非JSON文本消息：{msg[:50]}...")
            else:
                print(f"【接收提示】收到二进制消息（长度：{len(msg)}字节），忽略")

        except WebSocketException as e:
            print(f"【接收异常】连接中断：{str(e)}")
            self.close()
            return
        except OSError as e:
            # 捕获系统套接字错误
            print(f"【接收异常】系统套接字错误：{str(e)}")
            self.close()
            return
        except Exception as e:
            print(f"【接收异常】未知错误：{str(e)}")
            self.close()
            return

    def send_audio_frame(self, audio_frame):
        """发送单帧音频数据"""
        if not self.is_connected or not self.ws:
            print("【发送失败】WebSocket未连接")
            return False

        try:
            self.ws.send_binary(audio_frame)
            return True
        except WebSocketException as e:
            print(f"【发送失败】WebSocket连接中断：{str(e)}")
            self.close()
            return False
        except Exception as e:
            print(f"【发送异常】未知错误：{str(e)}")
            self.close()
            return False

    def send_end_signal(self):
        """发送结束标记"""
        if not self.is_connected or not self.ws:
            print("【发送失败】WebSocket未连接")
            return False

        try:
            end_msg = {"end": True}
            if self.session_id:
                end_msg["sessionId"] = self.session_id  # 关联当前会话
            end_msg_str = json.dumps(end_msg, ensure_ascii=False)
            self.ws.send(end_msg_str)
            print(f"【发送结束】已发送标准JSON结束标记：{end_msg_str}")
            return True
        except WebSocketException as e:
            print(f"【发送失败】WebSocket连接中断：{str(e)}")
            self.close()
            return False
        except Exception as e:
            print(f"【发送异常】未知错误：{str(e)}")
            self.close()
            return False

    def get_recognized_text(self):
        """获取识别出的文本"""
        return self.recognized_text

    def clear_recognized_text(self):
        """清空识别出的文本"""
        self.recognized_text = ""

    def close(self):
        """安全关闭WebSocket连接（增加状态保护）"""
        if self.is_connected and self.ws:
            self.is_connected = False
            try:
                # 先判断连接是否仍处于打开状态
                if self.ws.connected:
                    self.ws.close(status=1000, reason="客户端正常关闭")
                print("【连接关闭】WebSocket已安全关闭")
            except Exception as e:
                print(f"【关闭异常】关闭时出错：{str(e)}")
        else:
            print("【连接关闭】WebSocket已断开或未初始化")
