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
            atexit.register(self._shutdown)

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
