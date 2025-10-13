#!/usr/bin/env python3
# coding: utf-8
# Copyright: lejurobot 2025

"""
开关切换组件

本模块提供一个简单的开关切换组件，用于管理布尔状态并提供事件通知功能。

"""

from typing import Callable, List
import time
import threading

class ToggleEvent:
    """开关事件数据结构"""
    def __init__(self, switch_name: str, old_state: bool, new_state: bool, timestamp: float):
        self.switch_name = switch_name
        self.old_state = old_state
        self.new_state = new_state
        self.timestamp = timestamp
    
    def __repr__(self):
        return f"ToggleEvent({self.switch_name}: {self.old_state} -> {self.new_state})"

class ToggleSwitch:
    """开关类 - 管理布尔状态的切换"""
    
    def __init__(self, name: str, initial_state: bool = False):
        """
        初始化开关
        
        Args:
            name: 开关名称
            initial_state: 初始状态 (默认False)
        """
        self.name = name
        self.state = initial_state
        self.last_toggle_time = 0.0
        self.toggle_count = 0
        self.callbacks: List[Callable[[ToggleEvent], None]] = []
        self._lock = threading.Lock()
    
    def toggle(self) -> bool:
        """
        切换开关状态
        
        Returns:
            bool: 切换后的状态
        """
        with self._lock:
            old_state = self.state
            self.state = not self.state
            self.last_toggle_time = time.time()
            self.toggle_count += 1
            
            # 触发事件
            event = ToggleEvent(self.name, old_state, self.state, self.last_toggle_time)
            self._notify_callbacks(event)
            
            return self.state
    
    def set_state(self, new_state: bool) -> bool:
        """
        设置开关状态
        
        Args:
            new_state: 新状态
            
        Returns:
            bool: 设置后的状态
        """
        with self._lock:
            if self.state != new_state:
                old_state = self.state
                self.state = new_state
                self.last_toggle_time = time.time()
                self.toggle_count += 1
                
                # 触发事件
                event = ToggleEvent(self.name, old_state, self.state, self.last_toggle_time)
                self._notify_callbacks(event)
            
            return self.state
    
    def turn_on(self) -> bool:
        """打开开关"""
        return self.set_state(True)
    
    def turn_off(self) -> bool:
        """关闭开关"""
        return self.set_state(False)
    
    
    def add_callback(self, callback: Callable[[ToggleEvent], None]):
        """
        添加状态变化回调函数
        
        Args:
            callback: 回调函数，接收ToggleEvent参数
        """
        with self._lock:
            self.callbacks.append(callback)
    
    def remove_callback(self, callback: Callable[[ToggleEvent], None]):
        """
        移除回调函数
        
        Args:
            callback: 要移除的回调函数
        """
        with self._lock:
            if callback in self.callbacks:
                self.callbacks.remove(callback)
    
    def _notify_callbacks(self, event: ToggleEvent):
        """通知所有回调函数"""
        # 创建回调列表的副本，避免在遍历过程中列表被修改
        callbacks_copy = self.callbacks.copy()
        for callback in callbacks_copy:
            try:
                callback(event)
            except Exception as e:
                print(f"Error in toggle switch callback: {e}")
    
    def get_stats(self) -> dict:
        """
        获取开关统计信息
        
        Returns:
            dict: 统计信息
        """
        with self._lock:
            return {
                'name': self.name,
                'state': self.state,
                'toggle_count': self.toggle_count,
                'last_toggle_time': self.last_toggle_time,
                'callbacks_count': len(self.callbacks)
            }
    
    def reset_stats(self):
        """重置统计信息"""
        with self._lock:
            self.toggle_count = 0
            self.last_toggle_time = 0.0
    
    def __bool__(self) -> bool:
        """布尔转换，可以直接用 if 判断开关状态"""
        with self._lock:
            return self.state
    
    def __str__(self):
        with self._lock:
            return f"ToggleSwitch({self.name}: {self.state})"
    
    def __repr__(self):
        with self._lock:
            return f"ToggleSwitch(name='{self.name}', state={self.state})"

if __name__ == "__main__":
    """使用示例"""
    print("=== Toggle Switch 示例 ===\n")
    
    # 基本开关使用
    print("1. 基本开关使用:")
    hand_wrench = ToggleSwitch("hand_wrench")
    print(f"初始状态: {hand_wrench}")
    print(f"布尔判断: {bool(hand_wrench)}")
    print(f"直接if判断: {'开启' if hand_wrench else '关闭'}")
    
    # 添加回调
    def on_hand_wrench_changed(event: ToggleEvent):
        print(f"- 末端力施加状态变化: {event.old_state} -> {event.new_state}")
    
    hand_wrench.add_callback(on_hand_wrench_changed)
    
    # 切换开关
    hand_wrench.toggle()
    print(f"切换后状态: {hand_wrench}")
    print(f"布尔判断: {bool(hand_wrench)}")
    print(f"直接if判断: {'开启' if hand_wrench else '关闭'}")
    
    hand_wrench.toggle()
    print(f"切换后状态: {hand_wrench}")
    print(f"布尔判断: {bool(hand_wrench)}")
    print(f"直接if判断: {'开启' if hand_wrench else '关闭'}")
    
    # 统计信息
    print(f"\n3. 统计信息:")
    stats = hand_wrench.get_stats()
    print(f"  开关名称: {stats['name']}")
    print(f"  当前状态: {stats['state']}")
    print(f"  切换次数: {stats['toggle_count']}")
    print(f"  最后切换时间: {stats['last_toggle_time']}")
    print(f"  回调数量: {stats['callbacks_count']}")
    
    # 线程安全测试
    print(f"\n4. 线程安全测试:")
    import threading
    import time
    
    test_switch = ToggleSwitch("thread_test")
    counter = [0]  # 用于计数器
    
    def switch_operator(name: str, operations: int):
        """开关操作线程"""
        for i in range(operations):
            test_switch.toggle()
            time.sleep(0.001)  # 短暂延迟
            counter[0] += 1
    
    def switch_reader(name: str, operations: int):
        """开关读取线程"""
        for i in range(operations):
            state = bool(test_switch)
            stats = test_switch.get_stats()
            time.sleep(0.001)  # 短暂延迟
            counter[0] += 1
    
    # 创建多个线程
    threads = []
    threads.append(threading.Thread(target=switch_operator, args=("operator1", 10)))
    threads.append(threading.Thread(target=switch_operator, args=("operator2", 10)))
    threads.append(threading.Thread(target=switch_reader, args=("reader1", 10)))
    threads.append(threading.Thread(target=switch_reader, args=("reader2", 10)))
    
    # 启动所有线程
    for thread in threads:
        thread.start()
    
    # 等待所有线程完成
    for thread in threads:
        thread.join()
    
    final_stats = test_switch.get_stats()
    print(f"  最终状态: {final_stats['state']}")
    print(f"  总切换次数: {final_stats['toggle_count']}")
    print(f"  总操作次数: {counter[0]}")
    print(f"  剩余回调数量: {final_stats['callbacks_count']}")
    
    print("\n✅ Toggle Switch 示例完成！")