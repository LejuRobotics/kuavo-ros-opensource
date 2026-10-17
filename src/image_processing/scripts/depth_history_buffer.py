#!/usr/bin/env python3
"""Build the policy's timestamp-aligned history from compact depth frames."""

import time
from collections import deque

import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray, String


class DepthHistoryBuffer:
    def __init__(self):
        rospy.init_node("depth_history_buffer")
        self.input_topic = rospy.get_param(
            "~input_topic", "/waist_camera/depth/image_enhanced_compact"
        )
        self.width = int(rospy.get_param("~output_width", 64))
        self.height = int(rospy.get_param("~output_height", 36))
        self.length = max(1, int(rospy.get_param("~history_length", 8)))
        self.interval = max(0.0, float(rospy.get_param("~history_interval_sec", 0.1)))
        self.keep_sec = self.interval * (self.length - 1) + 1.0
        self.stats_interval = max(0.5, float(rospy.get_param("~stats_interval_sec", 5.0)))
        self.history = deque()
        self.bridge = CvBridge()
        self.received = 0
        self.last_report = time.monotonic()

        self.history_pub = rospy.Publisher(
            rospy.get_param("~history_topic", "/camera/depth/depth_history_array"),
            Float64MultiArray,
            queue_size=1,
        )
        self.stamps_pub = rospy.Publisher(
            rospy.get_param("~history_stamps_topic", "/camera/depth/depth_history_stamps"),
            Float64MultiArray,
            queue_size=1,
        )
        self.ready_pub = rospy.Publisher(
            rospy.get_param("~history_ready_topic", "/camera/depth/history_ready"),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.diagnostics_pub = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "/camera/depth/postprocess_diagnostics"),
            String,
            queue_size=1,
            latch=True,
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Image,
            self.callback,
            queue_size=1,
            buff_size=2**20,
            tcp_nodelay=True,
        )

    def select_history(self, stamp_sec, image):
        if self.history and stamp_sec < self.history[-1][0]:
            rospy.logwarn_throttle(5.0, "Compact depth timestamps moved backwards; resetting history")
            self.history.clear()
        if self.history and stamp_sec == self.history[-1][0]:
            self.history[-1] = (stamp_sec, image.copy())
        else:
            self.history.append((stamp_sec, image.copy()))

        oldest_needed = stamp_sec - self.keep_sec
        while len(self.history) > 1 and self.history[1][0] < oldest_needed:
            self.history.popleft()

        available = list(self.history)
        targets = [
            stamp_sec - self.interval * (self.length - 1 - index)
            for index in range(self.length)
        ]
        return [min(available, key=lambda item: abs(item[0] - target)) for target in targets]

    def callback(self, msg):
        started = time.monotonic()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "Compact depth decode failed: %s", error)
            return

        if image.shape != (self.height, self.width):
            rospy.logerr_throttle(
                2.0,
                "Compact depth shape mismatch: actual=%s expected=(%d, %d)",
                image.shape,
                self.height,
                self.width,
            )
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        selected = self.select_history(stamp.to_sec(), np.asarray(image, dtype=np.float32))
        stack = np.stack([item[1] for item in selected]).reshape(-1).astype(np.float64)

        history_msg = Float64MultiArray()
        history_msg.data = stack.tolist()
        self.history_pub.publish(history_msg)

        stamps_msg = Float64MultiArray()
        stamps_msg.data = [item[0] for item in selected]
        self.stamps_pub.publish(stamps_msg)

        target_span = self.interval * (self.length - 1)
        available_span = self.history[-1][0] - self.history[0][0]
        ready = available_span >= max(0.0, target_span - max(0.05, self.interval * 0.25))
        self.ready_pub.publish(Bool(data=ready))
        self.received += 1

        now = time.monotonic()
        if now - self.last_report >= self.stats_interval:
            selected_span = selected[-1][0] - selected[0][0]
            age_ms = (rospy.Time.now() - stamp).to_sec() * 1000.0
            diagnostic = (
                f"compact_received={self.received} buffer_ms={(now-started)*1000.0:.3f} "
                f"capture_to_history_ms={age_ms:.3f} compact_shape={self.height}x{self.width} "
                f"history_selected_span_sec={selected_span:.3f} "
                f"history_target_span_sec={target_span:.3f} history_ready={ready}"
            )
            self.diagnostics_pub.publish(String(data=diagnostic))
            rospy.loginfo(diagnostic)
            self.last_report = now


if __name__ == "__main__":
    DepthHistoryBuffer()
    rospy.spin()
