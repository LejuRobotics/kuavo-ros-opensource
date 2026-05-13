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
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import ssl
import traceback


class XFYunTTSClient:
    def __init__(self, app_id, api_key, api_secret, base_url=None):
        self.app_id = app_id
        self.api_key = api_key
        self.api_secret = api_secret
        self.base_ws_url = (
            base_url
            if base_url
            else "wss://tts-api.xfyun.cn/v2/tts"
        )
        self.ws = None
        self.is_connected = False
        self.recv_thread = None
        self.audio_buffer = []
        self._synthesis_completed = False

    def _generate_auth_params(self):
        """生成鉴权参数"""
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
        
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(self.api_secret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.api_key, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        
        # 将请求的鉴权参数组合为字典
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        
        return v

    def connect(self):
        """建立WebSocket连接"""
        try:
            auth_params = self._generate_auth_params()
            params_str = urllib.parse.urlencode(auth_params)
            full_ws_url = f"{self.base_ws_url}?{params_str}"
            print(f"【TTS连接】完整URL：{full_ws_url}")

            # 初始化WebSocket连接
            self.ws = create_connection(
                full_ws_url, timeout=15, enable_multithread=True
            )
            self.is_connected = True
            print("【TTS连接成功】WebSocket握手完成，等待服务端就绪...")
            time.sleep(0.5)
            return True
        except WebSocketException as e:
            print(f"【TTS连接失败】WebSocket错误：{str(e)}")
            if hasattr(e, "status_code"):
                print(f"【TTS服务端状态码】{e.status_code}")
            return False
        except Exception as e:
            print(f"【TTS连接异常】其他错误：{str(e)}")
            return False

    def send_text(self, text):
        """发送文本数据"""
        if not self.is_connected or not self.ws:
            print("【TTS发送失败】WebSocket未连接")
            return False

        try:
            # 公共参数(common)
            common_args = {"app_id": self.app_id}
            # 业务参数(business)
            business_args = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "x4_yezi", "tte": "utf8","volume":100}
            # 数据参数(data)
            data = {"status": 2, "text": str(base64.b64encode(text.encode('utf-8')), "UTF8")}
            
            # 构建发送数据
            send_data = {
                "common": common_args,
                "business": business_args,
                "data": data
            }
            
            send_str = json.dumps(send_data, ensure_ascii=False)
            self.ws.send(send_str)
            print(f"【TTS发送文本】已发送文本数据：{text}")
            return True
        except WebSocketException as e:
            print(f"【TTS发送失败】WebSocket连接中断：{str(e)}")
            self.close()
            return False
        except Exception as e:
            print(f"【TTS发送异常】未知错误：{str(e)}")
            self.close()
            return False

    def recv_audio_frame(self):
        """接收音频帧"""
        if not self.is_connected or not self.ws:
            print("【TTS接收失败】WebSocket未连接")
            return None

        try:
            msg = self.ws.recv()
            if not msg:
                print("【TTS接收消息】服务端关闭连接")
                self.close()
                return None

            if isinstance(msg, str):
                try:
                    if not msg or msg == "":
                        return None
                    msg_json = json.loads(msg)
                    print(f"【TTS接收消息】")

                    code = msg_json.get("code")
                    if code != 0:
                        errMsg = msg_json.get("message")
                        print(f"【TTS错误】代码：{code}，消息：{errMsg}")
                        return None
                    
                    # 检查是否合成完成
                    status = msg_json["data"]["status"]
                    if status == 2:
                        print("【TTS合成完成】已收到最终音频数据")
                        self._synthesis_completed = True

                    # 获取音频数据
                    if "data" in msg_json and "audio" in msg_json["data"]:
                        audio = msg_json["data"]["audio"]
                        if not audio:
                            print("【TTS接收音频】音频数据为空")
                            return None 
                        audio_bytes = base64.b64decode(audio)
                        

                        
                        return audio_bytes

                except json.JSONDecodeError:
                    print(f"【TTS接收异常】非JSON文本消息：{msg[:50]}...")
            else:
                print(f"【TTS接收提示】收到二进制消息（长度：{len(msg)}字节），忽略")

        except WebSocketException as e:
            print(f"【TTS接收异常】连接中断：{str(e)}")
            self.close()
            return None
        except OSError as e:
            print(f"【TTS接收异常】系统套接字错误：{str(e)}")
            self.close()
            return None
        except Exception as e:
            print(f"【TTS接收异常】未知错误：{str(e)}")
            print(traceback.format_exc())
            self.close()
            return None

        return None

    def is_synthesis_finished(self):
        """检查是否合成完成"""
        return self._synthesis_completed
    
    def set_synthesis_flag(self,flag:bool):
        self._synthesis_completed = flag

    def close(self):
        """安全关闭WebSocket连接"""
        if self.is_connected and self.ws:
            self.is_connected = False
            try:
                if self.ws.connected:
                    self.ws.close(status=1000, reason="客户端正常关闭")
                print("【TTS连接关闭】WebSocket已安全关闭")
            except Exception as e:
                print(f"【TTS关闭异常】关闭时出错：{str(e)}")
        else:
            print("【TTS连接关闭】WebSocket已断开或未初始化")
