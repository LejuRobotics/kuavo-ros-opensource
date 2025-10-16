import asyncio
import websockets
import logging
import json
from datetime import datetime

logging.basicConfig(level=logging.INFO, format='[8889] %(asctime)s - %(levelname)s - %(message)s')

class WebSocketLoggerServer:
    _instance = None
    _initialized = False
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, host="0.0.0.0", port=8889):
        if self._initialized:
            return
        self.host = host
        self.port = port
        self.clients = set()
        self.queue = None
        self.server = None
        self.queue_ready = asyncio.Event()  # 新增：队列就绪事件
        self._initialized = True
        logging.info(f"WebSocketLoggerServer 初始化完成")
    
    async def _handler(self, websocket, path):
        self.clients.add(websocket)
        client_id = id(websocket)
        logging.info(f"[LoggerServer] 新客户端连接: {websocket.remote_address} (ID: {client_id})，当前连接数: {len(self.clients)}")

        try:
            await websocket.send(json.dumps({"type": "welcome", "message": "已经成功连接到日志服务器"}, ensure_ascii=False))
        except Exception as e:
            logging.warning(f"[LoggerServer] 欢迎消息发送失败: {e}")

        try:
            async for message in websocket:
                logging.debug(f"[LoggerServer] 收到客户端消息: {message}")
                await self.log(message)
        except websockets.exceptions.ConnectionClosed as e:
            logging.warning(f"[LoggerServer] 客户端连接断开: {websocket.remote_address}，原因: {e}")
            self.handle_disconnect(websocket)
        except Exception as e:
            self.handle_error(websocket, e)

  
    def handle_disconnect(self, websocket):
        client_id = id(websocket)
        if websocket in self.clients:
            self.clients.remove(websocket)  # 确保从集合中移除
            logging.info(f"[LoggerServer] 客户端断开连接: {websocket.remote_address} (ID: {client_id})，当前连接数: {len(self.clients)}")
        else:
            logging.warning(f"[LoggerServer] 客户端已不存在: {websocket.remote_address} (ID: {client_id})")
    
    def handle_error(self, websocket, error):
        logging.error(f"[LoggerServer] 客户端通信错误: {websocket.remote_address}，错误: {error}")
        self.handle_disconnect(websocket)


    
    async def _log_broadcast_loop(self):
        logging.info("日志广播循环已启动")
        while True:
            try:
                msg = await asyncio.wait_for(self.queue.get(), timeout=1.0)

                logging.info(f"收到日志消息，准备广播给 {len(self.clients)} 个客户端: {msg[:100]}...")

                if self.clients:
                    # ✅ msg 本身是 JSON 字符串，直接广播，不要封装
                    for client in self.clients.copy():
                        try:
                            await client.send(msg)  # ✅ 直接发送客户端传过来的 JSON 字符串
                        except websockets.exceptions.ConnectionClosed:
                            self.clients.discard(client)
                        except Exception as e:
                            logging.error(f"发送日志给客户端失败: {e}")
                    logging.info(f"成功广播日志给 {len(self.clients)} 个客户端")
                self.queue.task_done()

            except asyncio.TimeoutError:
                await asyncio.sleep(0.1)
            except Exception as e:
                logging.error(f"日志广播循环出错: {e}")
                await asyncio.sleep(1)

    
    async def start(self):
        logging.info(f"启动 WebSocket 日志服务 ws://{self.host}:{self.port}")
        self.queue = asyncio.Queue()
        self.queue_ready.set()  # 标记队列初始化完成
        
        broadcast_task = asyncio.create_task(self._log_broadcast_loop())
        
        try:
            self.server = await websockets.serve(
                self._handler, 
                self.host, 
                self.port,
                ping_interval=20,
                ping_timeout=10,
                close_timeout=10
            )
            logging.info(f"WebSocket 服务器已启动在 {self.host}:{self.port}")
            await self.server.wait_closed()
        except Exception as e:
            logging.error(f"WebSocket 服务器启动失败: {e}")
            broadcast_task.cancel()
            raise
    
    async def log(self, msg: str):
        if self.queue is None:
            logging.warning("队列未初始化，无法发送日志")
            return
        try:
            await self.queue.put(msg)
            logging.debug(f"日志已加入队列: {msg[:50]}...")
        except Exception as e:
            logging.error(f"添加日志到队列失败: {e}")
    
    async def stop(self):
        logging.info("正在停止 WebSocket 日志服务器...")
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        logging.info("WebSocket 日志服务器已停止")

# 全局唯一实例
logger = WebSocketLoggerServer()
