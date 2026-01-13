#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from datetime import datetime
from network_monitor.msg import NetworkPingMsg

class NetworkServerTopic:
    def __init__(self):
        rospy.init_node('network_server_topic', anonymous=True)
        
        # 创建发布者和订阅者
        self.ping_sub = rospy.Subscriber('/network_ping_request', NetworkPingMsg, self.ping_callback)
        self.pong_pub = rospy.Publisher('/network_ping_response', NetworkPingMsg, queue_size=10)
        
        # 统计变量
        self.request_count = 0
        self.start_time = time.time()
        
        rospy.loginfo("网络监控服务端启动 (Topic模式)，等待客户端连接...")
        
    def format_timestamp(self, timestamp):
        """将时间戳转换为可读格式"""
        dt = datetime.fromtimestamp(timestamp)
        return dt.strftime("%H:%M:%S.%f")[:-3]
        
    def ping_callback(self, msg):
        """处理ping请求"""
        self.request_count += 1
        current_time = time.time()
        
        # 简化日志输出
        rospy.loginfo(f"收到 #{msg.request_id} | 客户端时间: {self.format_timestamp(msg.timestamp)} | 服务端时间: {self.format_timestamp(current_time)}")
        
        # 创建响应消息
        response_msg = NetworkPingMsg()
        response_msg.request_id = msg.request_id
        response_msg.timestamp = msg.timestamp  # 返回客户端发送的时间戳
        response_msg.success = True
        response_msg.message = f"请求编号: {self.request_count}"
        
        # 发布响应
        self.pong_pub.publish(response_msg)
    
    def run(self):
        """运行服务端"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        server = NetworkServerTopic()
        server.run()
    except rospy.ROSInterruptException:
        pass
