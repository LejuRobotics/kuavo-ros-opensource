#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =============================================================================
# 用户配置参数 - 可根据需要修改以下参数
# =============================================================================

# 网络监控配置
FREQUENCY = 100                   # 发送频率 (Hz)
TIMEOUT = 1.0                     # 请求超时时间 (秒)
INTERVAL = 3600                   # 阶段报告间隔 (秒)

# 文件生成配置
GENERATE_HOURLY_FILES = True      # 是否生成单独的阶段报告文件
GENERATE_FINAL_FILE = True        # 是否生成单独的最终报告文件

# =============================================================================

import rospy
import time
import signal
import sys
import threading
import os
import csv
from datetime import datetime
from std_msgs.msg import Float64, Bool, String
from network_monitor.msg import NetworkPingMsg

class NetworkClientTopic:
    def __init__(self):
        rospy.init_node('network_client_topic', anonymous=True)
        
        # 创建发布者和订阅者
        self.ping_pub = rospy.Publisher('/network_ping_request', NetworkPingMsg, queue_size=10)
        self.pong_sub = rospy.Subscriber('/network_ping_response', NetworkPingMsg, self.pong_callback)
        
        # 统计变量
        self.request_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.timeout_count = 0
        self.connection_lost_count = 0
        self.total_latency = 0.0
        self.max_latency = 0.0
        self.min_latency = float('inf')
        
        # 时间记录
        self.start_time = time.time()
        self.last_success_time = time.time()
        
        # 断连检测变量
        self.connection_lost_periods = []  # 存储断连时间段
        self.connection_lost_start = None  # 当前断连开始时间
        self.is_connected = True  # 当前连接状态
        self.consecutive_timeouts = 0  # 连续超时计数
        
        # 超时设置
        self.timeout_seconds = TIMEOUT
        self.pending_requests = {}  # 存储待响应的请求
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parent4 = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..'))
        if os.path.basename(parent4) == 'network_monitor':
            self.base_output_dir = os.path.join(parent4, 'output')
        else:
            self.base_output_dir = os.path.join(parent4, 'network_monitor', 'output')

        # 文件输出设置
        self.setup_file_output()
        
        # CSV数据记录设置
        self.csv_data = []  # 存储当前阶段的CSV数据
        self.csv_file = None  # 当前CSV文件路径
        self.csv_start_time = self.start_time  # CSV文件开始时间
        self.setup_csv_file()
        
        # 定期报告设置
        self.stage_report_interval = INTERVAL
        self.last_stage_report_time = self.start_time
        self.stage_stats = {
            'requests': 0,
            'successes': 0,
            'failures': 0,
            'timeouts': 0,
            'total_latency': 0.0,
            'max_latency': 0.0,
            'min_latency': float('inf')
        }
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 等待订阅者连接
        rospy.loginfo("等待网络监控服务...")
        rospy.sleep(1.0)  # 给服务端时间启动
        
        rospy.loginfo("网络监控客户端启动 (Topic模式)")
        self.log_to_file("网络监控客户端启动 (Topic模式)")
        
    def setup_file_output(self):
        """设置文件输出"""
        # 创建输出目录结构到 newcheck_tool/network_monitor/output
        output_dir = self.base_output_dir
        log_dir = os.path.join(output_dir, "logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 使用启动时间创建日志文件名
        start_dt = datetime.fromtimestamp(self.start_time)
        log_filename = f"network_monitor_{start_dt.strftime('%Y%m%d_%H%M%S')}.log"
        report_filename = f"network_monitor_reports_{start_dt.strftime('%Y%m%d_%H%M%S')}.txt"
        
        self.log_file = os.path.join(log_dir, log_filename)
        self.report_file = os.path.join(log_dir, report_filename)
        
        # 初始化日志文件
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(f"\n{'='*80}\n")
            f.write(f"网络监控客户端启动 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*80}\n")
        
        # 初始化报告文件
        with open(self.report_file, 'a', encoding='utf-8') as f:
            f.write(f"\n{'='*80}\n")
            f.write(f"网络监控报告 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*80}\n")
    
    def setup_csv_file(self):
        """设置CSV文件"""
        # 创建CSV目录到 newcheck_tool/network_monitor/output
        output_dir = self.base_output_dir
        csv_dir = os.path.join(output_dir, "csv")
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        
        # 使用开始时间创建CSV文件名
        start_dt = datetime.fromtimestamp(self.csv_start_time)
        csv_filename = f"network_data_{start_dt.strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = os.path.join(csv_dir, csv_filename)
        
        # 初始化CSV文件，写入表头
        try:
            with open(self.csv_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['时间戳', '日期时间', '请求ID', '延迟(ms)', '状态', '成功率(%)', '平均延迟(ms)', '连接状态'])
            
            self.log_to_file(f"CSV文件已创建: {self.csv_file}")
        except Exception as e:
            rospy.logerr(f"创建CSV文件失败: {e}")
    
    def save_csv_data(self, req_id, latency, status, success_rate, avg_latency, connection_status):
        """保存数据到CSV"""
        current_time = time.time()
        dt = datetime.fromtimestamp(current_time)
        
        csv_row = [
            current_time,  # 时间戳
            dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],  # 可读时间
            req_id,  # 请求ID
            f"{latency:.2f}" if latency is not None else "N/A",  # 延迟
            status,  # 状态
            f"{success_rate:.1f}" if success_rate is not None else "N/A",  # 成功率
            f"{avg_latency:.2f}" if avg_latency is not None else "N/A",  # 平均延迟
            connection_status  # 连接状态
        ]
        
        self.csv_data.append(csv_row)
        
        # 实时写入CSV文件
        try:
            with open(self.csv_file, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(csv_row)
        except Exception as e:
            rospy.logerr(f"写入CSV文件失败: {e}")
    
    def save_stage_csv(self):
        """保存当前阶段的CSV数据并创建新文件"""
        if not self.csv_data:
            return
        
        # 使用开始和结束时间命名文件
        start_dt = datetime.fromtimestamp(self.csv_start_time)
        end_time = time.time()
        end_dt = datetime.fromtimestamp(end_time)
        
        saved_filename = f"network_data_{start_dt.strftime('%Y%m%d_%H%M%S')}_to_{end_dt.strftime('%Y%m%d_%H%M%S')}.csv"
        saved_file = os.path.join(self.base_output_dir, "csv", saved_filename)
        
        try:
            # 将数据写入完成文件
            with open(saved_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['时间戳', '日期时间', '请求ID', '延迟(ms)', '状态', '成功率(%)', '平均延迟(ms)', '连接状态'])
                writer.writerows(self.csv_data)
            
            self.log_to_file(f"阶段CSV数据已保存: {saved_file} (共{len(self.csv_data)}条记录)")
            
            # 删除进行中的文件（如果存在）
            if os.path.exists(self.csv_file):
                try:
                    os.remove(self.csv_file)
                    self.log_to_file(f"已删除进行中文件: {self.csv_file}")
                except Exception as e:
                    rospy.logwarn(f"删除进行中文件失败: {e}")
            
            # 清空当前数据
            self.csv_data = []
            
            # 更新开始时间并创建新的CSV文件
            self.csv_start_time = end_time
            self.setup_csv_file()
            
        except Exception as e:
            rospy.logerr(f"保存小时CSV数据失败: {e}")
    
    def log_to_file(self, message):
        """将消息写入日志文件"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {message}\n"
        
        try:
            with open(self.log_file, 'a', encoding='utf-8') as f:
                f.write(log_entry)
        except Exception as e:
            rospy.logerr(f"写入日志文件失败: {e}")
        
    def format_timestamp(self, timestamp):
        """将时间戳转换为可读格式"""
        dt = datetime.fromtimestamp(timestamp)
        return dt.strftime("%H:%M:%S.%f")[:-3]  # 保留毫秒后3位
    
    def pong_callback(self, msg):
        """处理pong响应"""
        req_id = msg.request_id
        current_time = time.time()
        
        if req_id in self.pending_requests:
            req_data = self.pending_requests.pop(req_id)
            latency = (current_time - req_data['send_time']) * 1000  # 转换为毫秒
            
            if msg.success:
                self.success_count += 1
                self.total_latency += latency
                self.max_latency = max(self.max_latency, latency)
                self.min_latency = min(self.min_latency, latency)
                self.last_success_time = current_time
                
                # 检查连接恢复
                if not self.is_connected:
                    self.is_connected = True
                    if self.connection_lost_start:
                        lost_duration = current_time - self.connection_lost_start
                        # 记录所有断连，无论时长多短
                        self.connection_lost_periods.append({
                            'start': self.connection_lost_start,
                            'end': current_time,
                            'duration': lost_duration
                        })
                        rospy.loginfo(f"连接已恢复！断连时长: {lost_duration:.3f}秒")
                    self.connection_lost_start = None
                    self.consecutive_timeouts = 0  # 重置连续超时计数
                
                # 计算统计信息
                avg_latency = self.total_latency / self.success_count
                success_rate = (self.success_count / self.request_count) * 100
                
                # 输出格式
                status = "成功"
                connection_status = "正常" if self.is_connected else "断连"
                log_message = f"#{req_id} | 时间: {self.format_timestamp(req_data['send_time'])} | 延迟: {latency:.2f}ms | 成功率: {success_rate:.1f}% | 平均延迟: {avg_latency:.2f}ms | 状态: {connection_status}"
                rospy.loginfo(log_message)
                self.log_to_file(log_message)
                
                # 保存到CSV
                self.save_csv_data(req_id, latency, status, success_rate, avg_latency, connection_status)
                
                # 更新阶段统计
                self.update_stage_stats('success', latency)
                
            else:
                self.failure_count += 1
                self.connection_lost_count += 1
                error_message = f"#{req_id} 失败: {msg.message}"
                rospy.logwarn(error_message)
                self.log_to_file(error_message)
                
                # 保存到CSV
                status = "失败"
                connection_status = "正常" if self.is_connected else "断连"
                success_rate = (self.success_count / self.request_count) * 100 if self.request_count > 0 else 0
                avg_latency = self.total_latency / self.success_count if self.success_count > 0 else 0
                self.save_csv_data(req_id, None, status, success_rate, avg_latency, connection_status)
                
                # 更新阶段统计
                self.update_stage_stats('failure')
    
    def check_timeouts(self):
        """检查超时的请求"""
        current_time = time.time()
        timeout_requests = []
        
        for req_id, req_data in self.pending_requests.items():
            if current_time - req_data['send_time'] > self.timeout_seconds:
                timeout_requests.append(req_id)
        
        if timeout_requests:
            self.consecutive_timeouts += len(timeout_requests)
        else:
            self.consecutive_timeouts = 0  # 重置连续超时计数
        
        for req_id in timeout_requests:
            req_data = self.pending_requests.pop(req_id)
            self.timeout_count += 1
            self.connection_lost_count += 1
            
            timeout_message = f"#{req_id} 超时！发送时间: {self.format_timestamp(req_data['send_time'])} | 超时时间: {self.timeout_seconds}秒"
            rospy.logerr(timeout_message)
            self.log_to_file(timeout_message)
            
            # 保存到CSV
            status = "超时"
            connection_status = "正常" if self.is_connected else "断连"
            success_rate = (self.success_count / self.request_count) * 100 if self.request_count > 0 else 0
            avg_latency = self.total_latency / self.success_count if self.success_count > 0 else 0
            self.save_csv_data(req_id, None, status, success_rate, avg_latency, connection_status)
            
            # 更新阶段统计
            self.update_stage_stats('timeout')
            
            # 敏感的断连检测 - 连续2个超时就认为是断连
            if self.is_connected and self.consecutive_timeouts >= 2:
                self.is_connected = False
                self.connection_lost_start = current_time
                disconnect_message = "检测到连接丢失！"
                rospy.logerr(disconnect_message)
                self.log_to_file(disconnect_message)
    
    def send_ping(self):
        """发送请求"""
        self.request_count += 1
        send_time = time.time()
        req_id = self.request_count
        
        # 更新阶段统计
        self.update_stage_stats('request')
        
        # 检查是否需要生成阶段报告
        self.check_stage_report()
        
        # 检查超时请求
        self.check_timeouts()
        
        try:
            # 记录待响应请求
            self.pending_requests[req_id] = {
                'send_time': send_time,
                'timestamp': send_time
            }
            
            # 创建ping消息
            ping_msg = NetworkPingMsg()
            ping_msg.request_id = req_id
            ping_msg.timestamp = send_time
            ping_msg.message = f"Request #{req_id}"
            
            # 发布ping请求
            self.ping_pub.publish(ping_msg)
            
        except Exception as e:
            self.failure_count += 1
            self.connection_lost_count += 1
            if req_id in self.pending_requests:
                del self.pending_requests[req_id]
            
            # 检查是否断连
            if self.is_connected:
                self.is_connected = False
                self.connection_lost_start = time.time()
                disconnect_message = "检测到连接丢失！"
                rospy.logerr(disconnect_message)
                self.log_to_file(disconnect_message)
            
            error_message = f"#{req_id} 发送失败: {e}"
            rospy.logerr(error_message)
            self.log_to_file(error_message)
            
            # 更新阶段统计
            self.update_stage_stats('failure')
    
    def update_stage_stats(self, stat_type, latency=None):
        """更新阶段统计信息"""
        if stat_type == 'request':
            self.stage_stats['requests'] += 1
        elif stat_type == 'success':
            self.stage_stats['successes'] += 1
            if latency is not None:
                self.stage_stats['total_latency'] += latency
                self.stage_stats['max_latency'] = max(self.stage_stats['max_latency'], latency)
                self.stage_stats['min_latency'] = min(self.stage_stats['min_latency'], latency)
        elif stat_type == 'failure':
            self.stage_stats['failures'] += 1
        elif stat_type == 'timeout':
            self.stage_stats['timeouts'] += 1
    
    def check_stage_report(self):
        """检查是否需要生成阶段报告"""
        current_time = time.time()
        if current_time - self.last_stage_report_time >= self.stage_report_interval:
            self.generate_stage_report()
            self.save_stage_csv()  # 保存当前阶段的CSV数据
            self.reset_stage_stats()
            self.last_stage_report_time = current_time
    
    def generate_stage_report(self):
        """生成阶段报告"""
        current_time = time.time()
        stage_duration = current_time - self.last_stage_report_time
        
        # 计算统计信息
        if self.stage_stats['requests'] > 0:
            success_rate = (self.stage_stats['successes'] / self.stage_stats['requests']) * 100
            failure_rate = (self.stage_stats['failures'] / self.stage_stats['requests']) * 100
            timeout_rate = (self.stage_stats['timeouts'] / self.stage_stats['requests']) * 100
        else:
            success_rate = failure_rate = timeout_rate = 0
        
        if self.stage_stats['successes'] > 0:
            avg_latency = self.stage_stats['total_latency'] / self.stage_stats['successes']
        else:
            avg_latency = 0
        
        # 生成报告内容
        report_lines = []
        report_lines.append("===================== 阶段统计报告 =====================")
        report_lines.append(f"时间: {self.format_timestamp(self.last_stage_report_time)} - {self.format_timestamp(current_time)} | 时长: {stage_duration:.2f}秒 ({stage_duration/60:.1f}分钟)")
        report_lines.append(f"※ 请求统计: 总数{self.stage_stats['requests']} | 成功{self.stage_stats['successes']} | 失败{self.stage_stats['failures']} | 超时{self.stage_stats['timeouts']}")
        report_lines.append(f"※ 成功率统计: 成功{success_rate:.2f}% | 失败{failure_rate:.2f}% | 超时{timeout_rate:.2f}%")
        
        if self.stage_stats['successes'] > 0:
            report_lines.append(f"※ 延迟统计: 平均{avg_latency:.2f}ms | 最大{self.stage_stats['max_latency']:.2f}ms | 最小{self.stage_stats['min_latency']:.2f}ms")
        else:
            report_lines.append("※ 延迟统计: 无成功请求")
        
        report_lines.append("=" * 60)
        
        # 输出到控制台和文件
        for line in report_lines:
            rospy.loginfo(line)
            self.log_to_file(line)
        
        # 写入主报告文件
        try:
            with open(self.report_file, 'a', encoding='utf-8') as f:
                for line in report_lines:
                    f.write(line + "\n")
                f.write("\n")
        except Exception as e:
            rospy.logerr(f"写入报告文件失败: {e}")
        
        # 根据配置决定是否生成单独的阶段报告文件
        if GENERATE_HOURLY_FILES:
            start_dt = datetime.fromtimestamp(self.last_stage_report_time)
            end_dt = datetime.fromtimestamp(current_time)
            stage_report_filename = f"stage_report_{start_dt.strftime('%Y%m%d_%H%M%S')}_to_{end_dt.strftime('%Y%m%d_%H%M%S')}.txt"
            stage_report_file = os.path.join(self.base_output_dir, "logs", stage_report_filename)
            
            # 写入单独的阶段报告文件
            try:
                with open(stage_report_file, 'w', encoding='utf-8') as f:
                    f.write(f"网络监控阶段报告\n")
                    f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write("=" * 60 + "\n")
                    for line in report_lines:
                        f.write(line + "\n")
            except Exception as e:
                rospy.logerr(f"写入阶段报告文件失败: {e}")
    
    def reset_stage_stats(self):
        """重置阶段统计"""
        self.stage_stats = {
            'requests': 0,
            'successes': 0,
            'failures': 0,
            'timeouts': 0,
            'total_latency': 0.0,
            'max_latency': 0.0,
            'min_latency': float('inf')
        }
    
    def signal_handler(self, signum, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("\n收到退出信号，正在生成报告...")
        self.save_stage_csv()  # 保存最后的CSV数据
        self.generate_final_report()
        sys.exit(0)
    
    def merge_continuous_disconnects(self, disconnect_periods, gap_threshold=0.1):
        """
        合并连续的断连时段
        gap_threshold: 两个断连时段之间的最大间隔时间（秒），小于此值则认为是连续的
        """
        if not disconnect_periods:
            return []
        
        # 按开始时间排序
        sorted_periods = sorted(disconnect_periods, key=lambda x: x['start'])
        merged_periods = []
        current_period = sorted_periods[0].copy()
        
        for i in range(1, len(sorted_periods)):
            next_period = sorted_periods[i]
            # 如果下一个断连的开始时间与当前断连的结束时间之间的间隔小于阈值
            if next_period['start'] - current_period['end'] <= gap_threshold:
                # 合并断连时段
                current_period['end'] = next_period['end']
                current_period['duration'] = current_period['end'] - current_period['start']
            else:
                # 间隔太大，开始新的断连时段
                merged_periods.append(current_period)
                current_period = next_period.copy()
        
        # 添加最后一个时段
        merged_periods.append(current_period)
        return merged_periods

    def generate_final_report(self):
        """生成最终统计报告"""
        end_time = time.time()
        total_duration = end_time - self.start_time
        
        # 生成报告内容
        report_lines = []
        report_lines.append("===================== 网络监控客户端 - 最终统计报告 =====================")
        
        # 基本信息
        report_lines.append(f"时间: {self.format_timestamp(self.start_time)} - {self.format_timestamp(end_time)} | 运行时长: {total_duration:.2f}秒 ({total_duration/60:.1f}分钟)")
        
        # 请求统计
        report_lines.append(f"※ 请求统计: 总数{self.request_count} | 成功{self.success_count} | 失败{self.failure_count} | 超时{self.timeout_count} | 断连{self.connection_lost_count}")
        
        # 成功率统计
        if self.request_count > 0:
            success_rate = (self.success_count / self.request_count) * 100
            failure_rate = (self.failure_count / self.request_count) * 100
            timeout_rate = (self.timeout_count / self.request_count) * 100
            report_lines.append(f"※ 成功率统计: 成功{success_rate:.2f}% | 失败{failure_rate:.2f}% | 超时{timeout_rate:.2f}%")
        
        # 延迟统计
        if self.success_count > 0:
            avg_latency = self.total_latency / self.success_count
            report_lines.append(f"※ 延迟统计: 平均{avg_latency:.2f}ms | 最大{self.max_latency:.2f}ms | 最小{self.min_latency:.2f}ms")
        
        # 连接状态统计
        if self.connection_lost_periods:
            # 合并连续的断连时段
            merged_periods = self.merge_continuous_disconnects(self.connection_lost_periods, gap_threshold=0.1)
            total_lost_time = sum(period['duration'] for period in self.connection_lost_periods)
            report_lines.append(f"※ 连接状态: {'正常' if self.is_connected else '断连'} | 断连{len(merged_periods)}次 | 总断连时长{total_lost_time:.3f}秒 | 断连占比{(total_lost_time/total_duration)*100:.2f}%")
            report_lines.append("※ 断连详情:")
            for i, period in enumerate(merged_periods, 1):
                report_lines.append(f"  断连#{i}: {self.format_timestamp(period['start'])} - {self.format_timestamp(period['end'])} (时长: {period['duration']:.3f}秒)")
        else:
            report_lines.append(f"※ 连接状态: {'正常' if self.is_connected else '断连'} | 断连0次 | 无断连记录")
        
        report_lines.append("=" * 80)
        
        # 输出到控制台
        for line in report_lines:
            print(line)
        
        # 输出到日志文件
        for line in report_lines:
            self.log_to_file(line)
        
        # 写入主报告文件
        try:
            with open(self.report_file, 'a', encoding='utf-8') as f:
                f.write("\n")
                for line in report_lines:
                    f.write(line + "\n")
                f.write("\n")
        except Exception as e:
            rospy.logerr(f"写入最终报告文件失败: {e}")
        
        # 根据配置决定是否生成单独的最终报告文件
        if GENERATE_FINAL_FILE:
            start_dt = datetime.fromtimestamp(self.start_time)
            end_dt = datetime.fromtimestamp(end_time)
            final_report_filename = f"final_report_{start_dt.strftime('%Y%m%d_%H%M%S')}_to_{end_dt.strftime('%Y%m%d_%H%M%S')}.txt"
            final_report_file = os.path.join(self.base_output_dir, "logs", final_report_filename)
            
            # 写入单独的最终报告文件
            try:
                with open(final_report_file, 'w', encoding='utf-8') as f:
                    f.write(f"网络监控最终报告\n")
                    f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write("=" * 80 + "\n")
                    for line in report_lines:
                        f.write(line + "\n")
            except Exception as e:
                rospy.logerr(f"写入最终报告文件失败: {e}")
    
    def run(self):
        """运行客户端"""
        rate = rospy.Rate(FREQUENCY)  # 使用配置的频率
        
        rospy.loginfo(f"开始发送请求，频率: {FREQUENCY}Hz (Topic模式)")
        rospy.loginfo("按 Ctrl+C 退出并查看详细统计报告")
        rospy.loginfo("程序会自动处理网络断连，无需手动重启")
        rospy.loginfo(f"日志文件: {self.log_file}")
        rospy.loginfo(f"报告文件: {self.report_file}")
        rospy.loginfo(f"CSV文件: {self.csv_file}")
        rospy.loginfo(f"每{INTERVAL/60:.0f}分钟自动生成阶段报告和CSV数据")
        
        self.log_to_file(f"开始发送请求，频率: {FREQUENCY}Hz (Topic模式)")
        self.log_to_file(f"日志文件: {self.log_file}")
        self.log_to_file(f"报告文件: {self.report_file}")
        self.log_to_file(f"CSV文件: {self.csv_file}")
        self.log_to_file(f"每{INTERVAL/60:.0f}分钟自动生成阶段报告和CSV数据")
        
        try:
            while not rospy.is_shutdown():
                self.send_ping()
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            rospy.loginfo("收到键盘中断信号")
        except Exception as e:
            rospy.logerr(f"程序运行发生严重错误: {e}")
        
        # 如果正常退出，也生成报告
        self.generate_final_report()

if __name__ == '__main__':
    try:
        client = NetworkClientTopic()
        client.run()
    except rospy.ROSInterruptException:
        pass