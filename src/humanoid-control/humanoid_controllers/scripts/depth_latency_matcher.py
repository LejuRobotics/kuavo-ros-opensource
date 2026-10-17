#!/usr/bin/env python3
"""Delay/downsample native depth to match the LingBot enhancement pipeline."""

import math
import threading
from collections import deque

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DepthLatencyMatcher:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/waist_camera/depth/image_raw"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/waist_camera/depth/image_raw_latency_matched"
        )
        # At 60 Hz the newest eligible frame is, on average, another half frame
        # older than this threshold. 45 ms reproduces the enhanced pipeline's
        # measured ~54-55 ms capture-to-postprocess-output age.
        self.delay_sec = max(0.0, float(rospy.get_param("~delay_sec", 0.045)))
        self.rate_hz = max(0.1, float(rospy.get_param("~rate_hz", 22.0)))
        self.buffer_size = max(8, int(rospy.get_param("~buffer_size", 64)))
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/camera/depth/raw_latency_match_diagnostics"
        )

        self.lock = threading.Lock()
        self.frames = deque(maxlen=self.buffer_size)
        self.received = 0
        self.published = 0
        self.buffer_overflows = 0
        self.last_published_stamp = None
        self.last_report_time = rospy.Time.now()

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.diagnostics_publisher = rospy.Publisher(
            self.diagnostics_topic, String, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
        )
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.rate_hz), self.publish_timer
        )

        rospy.loginfo(
            "Native depth latency matcher: %s -> %s, delay=%.1f ms, rate=%.2f Hz",
            self.input_topic,
            self.output_topic,
            self.delay_sec * 1000.0,
            self.rate_hz,
        )

    def image_callback(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        with self.lock:
            self.received += 1
            if len(self.frames) == self.frames.maxlen:
                self.buffer_overflows += 1
            self.frames.append((stamp, msg))

    def publish_timer(self, _event):
        now = rospy.Time.now()
        target_stamp = now - rospy.Duration.from_sec(self.delay_sec)
        candidate = None

        with self.lock:
            # Use the newest frame that is old enough. Older eligible frames are
            # discarded, reproducing the enhancement worker's latest-frame rule.
            while self.frames and self.frames[0][0] <= target_stamp:
                candidate = self.frames.popleft()

        if candidate is None:
            return

        stamp, msg = candidate
        stamp_sec = stamp.to_sec()
        if self.last_published_stamp is not None and math.isclose(
            stamp_sec, self.last_published_stamp, rel_tol=0.0, abs_tol=1e-9
        ):
            return

        self.publisher.publish(msg)
        self.published += 1
        self.last_published_stamp = stamp_sec

        if (now - self.last_report_time).to_sec() >= 5.0:
            with self.lock:
                buffered = len(self.frames)
                received = self.received
                overflows = self.buffer_overflows
            source_age_ms = (now - stamp).to_sec() * 1000.0
            diagnostic = (
                f"received={received} published={self.published} buffered={buffered} "
                f"buffer_overflows={overflows} source_age_ms={source_age_ms:.3f} "
                f"target_delay_ms={self.delay_sec * 1000.0:.3f} "
                f"target_rate_hz={self.rate_hz:.3f}"
            )
            self.diagnostics_publisher.publish(String(data=diagnostic))
            rospy.loginfo(diagnostic)
            self.last_report_time = now


def main():
    rospy.init_node("depth_latency_matcher")
    DepthLatencyMatcher()
    rospy.spin()


if __name__ == "__main__":
    main()
