# test_ws_client.py
import asyncio
import websockets

async def main():
    uri = "ws://localhost:8889"
    async with websockets.connect(uri) as ws:
        while True:
            msg = await ws.recv()
            print("[收到消息]", msg)

asyncio.run(main())
