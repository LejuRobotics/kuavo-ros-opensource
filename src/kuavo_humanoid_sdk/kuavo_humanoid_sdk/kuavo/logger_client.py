# logger_client.py
import asyncio
import websockets
import json
import threading
import datetime
import inspect

# 全局单例对象
_logger_instance = None

def get_logger(timeout: int = 3):
    global _logger_instance
    if _logger_instance is None:
        _logger_instance = LoggerClient(timeout=timeout)
    return _logger_instance


class LoggerClient:
    def __init__(self, uri: str = None, timeout=3):
        # 强制使用 localhost
        self.uri = "ws://localhost:8889"
        print(f"[LoggerClient] 使用 localhost 构造连接地址: {self.uri}")
        
        self._loop = None
        self._ws = None
        self._connected_event = threading.Event()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        if not self._connected_event.wait(timeout=timeout + 2):
            print("[LoggerClient] 连接日志服务器超时")

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connect())
        self._loop.run_forever()

    async def _connect(self):
        try:
            self._ws = await websockets.connect(self.uri)
            print(f"[LoggerClient] 已连接到日志服务器 {self.uri}")
            self._connected_event.set()
            await self._listen()
        except Exception as e:
            print(f"[LoggerClient] 连接失败: {e}")

    async def _listen(self):
        try:
            async for _ in self._ws:
                pass
        except:
            pass

    def _format_log(self, msg: str, level: str = "INFO"):
        """格式化日志消息，获取模块和函数信息"""
        frame = inspect.currentframe().f_back.f_back  # 跳过 _format_log 和 send_log 的帧，获取实际调用者
        module = inspect.getmodule(frame)
        module_name = module.__name__ if module else "unknown_module"
        function_name = frame.f_code.co_name if frame else "unknown_function"
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        return {
            "level": level,
            "timestamp": timestamp,
            "message": msg,
            "module": module_name,
            "function": function_name
        }

    def _print_log(self, log_data: dict):
        print(f"[{log_data['timestamp']}] [{log_data['level']}] [{log_data['module']}.{log_data['function']}] {log_data['message']}")

    def send_log(self, msg: str, level: str = "INFO"):
        # 如果连接失败，直接输出到终端
        if not self._loop or not self._ws:
            log_data = self._format_log(msg, level)
            self._print_log(log_data)
            return

        async def _send():
            try:
                # 格式化日志数据
                log_data = self._format_log(msg, level)
                await self._ws.send(json.dumps(log_data, ensure_ascii=False))
            except Exception as e:
                # 如果发送失败，也输出到终端
                log_data = self._format_log(msg, level)
                print(f"[LoggerClient] 发送日志失败: {e}")
                self._print_log(log_data)

        asyncio.run_coroutine_threadsafe(_send(), self._loop)
