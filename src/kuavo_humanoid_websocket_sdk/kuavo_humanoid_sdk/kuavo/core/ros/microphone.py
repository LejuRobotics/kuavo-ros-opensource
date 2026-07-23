#!/usr/bin/env python3
# coding: utf-8
import collections
import threading
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
import roslibpy


class Microphone:
    """
    WebSocket-based microphone data subscription.
    Subscribes to an audio topic via roslibpy and buffers the received data.
    """
    def __init__(self, subscribe_topic="/micphone_data"):
        self._buffer = collections.deque()
        self._lock = threading.Lock()

        websocket = WebSocketKuavoSDK()
        self._topic = roslibpy.Topic(websocket.client, subscribe_topic, 'kuavo_msgs/AudioReceiverData')
        self._topic.subscribe(self._audio_callback)
        SDKLogger.debug(f"Microphone subscribed to topic: {subscribe_topic}")

    def _audio_callback(self, msg: dict):
        """
        Callback function for the audio subscriber. Appends data to the buffer.
        In D2, msg is a plain dict from roslibpy.
        """
        with self._lock:
            # msg['data'] is a list of ints in roslibpy; convert to bytes for compatibility
            data = msg.get('data', b'')
            if isinstance(data, list):
                data = bytes(data)
            self._buffer.append(data)

    def get_data(self):
        """
        Retrieves all data chunks from the buffer and clears it.
        This is designed to be called by the processing layer.
        """
        with self._lock:
            if not self._buffer:
                return None

            data_batch = b''.join(self._buffer)
            self._buffer.clear()
            return data_batch
