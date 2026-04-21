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
            

    def __del__(self):
        self.client.terminate()
        self.instance = None
