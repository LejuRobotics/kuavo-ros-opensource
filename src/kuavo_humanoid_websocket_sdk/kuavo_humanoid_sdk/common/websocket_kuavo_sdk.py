import time
import atexit
import roslibpy

class WebSocketKuavoSDK:

    _instance = None
    _initialized = False

    websocket_host = '127.0.0.1'
    websocket_port = 9090
    websocket_timeout = 5.0

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not self._initialized:
            self._initialized = True
            self.client = roslibpy.Ros(host=WebSocketKuavoSDK.websocket_host, port=WebSocketKuavoSDK.websocket_port)
            self.client.run(timeout=WebSocketKuavoSDK.websocket_timeout)
            self._optimize_transport()
            atexit.register(self._shutdown)

    def _optimize_transport(self):
        """降低 WebSocket 发送延迟：
        1. TCP_NODELAY: 禁用 Nagle 算法，小包不再等待凑满 MSS
        2. sync=True:   autobahn sendMessage 等待 TCP 写出完成再返回，
           消除 autobahn 内部发送队列引入的时序不确定性
        """
        try:
            proto = self.client.factory._proto
            if proto is None:
                return
            # 禁用 Nagle 算法：每个 send 立即发送，不等待凑满 TCP 段
            if proto.transport:
                proto.transport.setTcpNoDelay(True)
            # 替换 send_message，sync=True 保证每次发送顺序/时序确定
            import twisted.internet.reactor

            def _fast_send(payload):
                return twisted.internet.reactor.callFromThread(
                    proto.sendMessage, payload,
                    isBinary=False, fragmentSize=None,
                    sync=True, doNotCompress=False,
                )

            proto.send_message = _fast_send
        except Exception:
            pass  # 优化失败不影响功能，静默忽略

    def _shutdown(self):
        """Flush pending messages and close connection on process exit."""
        try:
            # Give the Twisted reactor time to flush pending WebSocket writes
            # before terminating. roslibpy.publish() is async (via
            # reactor.callFromThread), so messages may still be in the
            # reactor's event queue or transport write buffer at exit time.
            time.sleep(0.1)
            self.client.terminate()
        except Exception:
            pass

    def __del__(self):
        self.client.terminate()
        self.instance = None
