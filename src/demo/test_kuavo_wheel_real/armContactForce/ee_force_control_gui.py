#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""End-Effector Force Control GUI - Control left/right hand forces via sliders"""

import sys
import rospy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QPushButton, QGroupBox, QDoubleSpinBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from lb_force_ctrl import GRAVITY, LBForceController


class EEForceControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('End-Effector Force Control')
        self.setGeometry(1200, 100, 600, 600)
        rospy.init_node('ee_force_control_gui', anonymous=True)
        self.force_ctrl = LBForceController(wait_for_connection=True, connection_delay=0.1)
        self.force_empty_detact_enabled = True  # 当前挥空检测状态
        self.force_min, self.force_max, self.force_resolution = -6.0, 6.0, 1000
        self.gravity = GRAVITY
        self.forces = {'left': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'right': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        self.last_sent = {'left': {'x': None, 'y': None, 'z': None}, 'right': {'x': None, 'y': None, 'z': None}}
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        self.left_sliders, self.left_labels = {}, {}
        self.right_sliders, self.right_labels = {}, {}
        
        main_layout.addWidget(self.create_hand_group('Left Desire Force', 'left', '#4CAF50', '#45a049'))
        main_layout.addWidget(self.create_hand_group('Right Desire Force', 'right', '#2196F3', '#1976D2'))
        
        # 添加双手同步设置区域
        main_layout.addWidget(self.create_sync_force_group())
        
        button_layout = QHBoxLayout()
        button_layout.setSpacing(15)
        buttons = [('Reset All', self.reset_all_forces, '#f44336', '#da190b', '#c62828'),
                   ('Zero Left', self.zero_left_force, '#FF9800', '#F57C00', '#E65100'),
                   ('Zero Right', self.zero_right_force, '#2196F3', '#1976D2', '#1565C0')]
        for text, func, c1, c2, c3 in buttons:
            btn = QPushButton(text)
            btn.setFont(QFont('Arial', 13, QFont.Bold))
            btn.setStyleSheet(f"QPushButton{{background-color:{c1};color:white;border:none;padding:12px 24px;border-radius:6px;font-size:14px;font-weight:bold;min-width:120px;min-height:40px;}}QPushButton:hover{{background-color:{c2};}}QPushButton:pressed{{background-color:{c3};}}")
            btn.clicked.connect(func)
            button_layout.addWidget(btn)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        
        # 添加挥空检测开关按钮
        self.empty_detact_btn = QPushButton('Disable Force Empty Detect')
        self.empty_detact_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.empty_detact_btn.clicked.connect(self.toggle_force_empty_detact)
        self.empty_detact_btn.setStyleSheet("QPushButton{background-color:#4CAF50;color:white;border:none;padding:12px 24px;border-radius:6px;font-size:12px;font-weight:bold;min-width:200px;min-height:40px;}QPushButton:hover{background-color:#45a049;}QPushButton:pressed{background-color:#388E3C;}")
        empty_layout = QHBoxLayout()
        empty_layout.addStretch()
        empty_layout.addWidget(self.empty_detact_btn)
        empty_layout.addStretch()
        main_layout.addLayout(empty_layout)
        
        status = QLabel('Ready - Publishing to /desired_ee_force/left and /desired_ee_force/right')
        status.setAlignment(Qt.AlignCenter)
        status.setFont(QFont('Arial', 11))
        status.setStyleSheet("QLabel{background-color:#e3f2fd;padding:10px;border-radius:5px;color:#1976d2;}")
        main_layout.addWidget(status)
        
    def create_hand_group(self, title, hand, color, hover_color):
        group = QGroupBox(title)
        group.setFont(QFont('Arial', 13, QFont.Bold))
        layout = QVBoxLayout()
        layout.setSpacing(12)
        
        slider_style = f"""QSlider::groove:horizontal{{border:1px solid #999999;height:8px;background:#e0e0e0;border-radius:4px;}}QSlider::handle:horizontal{{background:{color};border:1px solid {color};width:20px;height:20px;margin:-6px 0;border-radius:10px;}}QSlider::handle:horizontal:hover{{background:{hover_color};}}"""
        
        for axis in ['x', 'y', 'z']:
            h_layout = QHBoxLayout()
            h_layout.setSpacing(15)
            
            label = QLabel(f'{axis.upper()}:')
            label.setMinimumWidth(50)
            label.setFont(QFont('Arial', 12, QFont.Bold))
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(self.force_min * self.force_resolution / 100), 
                          int(self.force_max * self.force_resolution / 100))
            slider.setValue(0)
            slider.setMinimumHeight(40)
            slider.setStyleSheet(slider_style)
            slider.valueChanged.connect(lambda v, a=axis, h=hand: self.on_slider_changed(h, a, v))
            
            value_label = QLabel('0.00 kg')
            value_label.setMinimumWidth(100)
            value_label.setAlignment(Qt.AlignRight)
            value_label.setFont(QFont('Arial', 11))
            
            sliders = self.left_sliders if hand == 'left' else self.right_sliders
            labels = self.left_labels if hand == 'left' else self.right_labels
            sliders[axis] = slider
            labels[axis] = value_label
            
            h_layout.addWidget(label)
            h_layout.addWidget(slider)
            h_layout.addWidget(value_label)
            layout.addLayout(h_layout)
        
        group.setLayout(layout)
        return group
    
    def create_sync_force_group(self):
        """创建双手同步设置力值的输入区域"""
        group = QGroupBox('Set Both Hands Force (Synchronized)')
        group.setFont(QFont('Arial', 13, QFont.Bold))
        layout = QVBoxLayout()
        layout.setSpacing(12)
        
        # 创建输入框
        input_layout = QHBoxLayout()
        input_layout.setSpacing(15)
        
        self.sync_spinboxes = {}
        for axis in ['x', 'y', 'z']:
            h_layout = QVBoxLayout()
            
            label = QLabel(f'{axis.upper()}:')
            label.setAlignment(Qt.AlignCenter)
            label.setFont(QFont('Arial', 11, QFont.Bold))
            h_layout.addWidget(label)
            
            spinbox = QDoubleSpinBox()
            spinbox.setRange(self.force_min, self.force_max)
            spinbox.setDecimals(2)
            spinbox.setSingleStep(1.0)
            spinbox.setValue(0.0)
            spinbox.setSuffix(' kg')
            spinbox.setFont(QFont('Arial', 11))
            spinbox.setStyleSheet("QDoubleSpinBox{border:2px solid #9E9E9E;border-radius:4px;padding:5px;background-color:white;}")
            self.sync_spinboxes[axis] = spinbox
            h_layout.addWidget(spinbox)
            
            input_layout.addLayout(h_layout)
        
        layout.addLayout(input_layout)
        
        # 添加发送按钮
        send_btn = QPushButton('Apply to Both Hands & Send')
        send_btn.setFont(QFont('Arial', 13, QFont.Bold))
        send_btn.setStyleSheet("QPushButton{background-color:#9C27B0;color:white;border:none;padding:12px 24px;border-radius:6px;font-size:14px;font-weight:bold;min-width:200px;min-height:45px;}QPushButton:hover{background-color:#7B1FA2;}QPushButton:pressed{background-color:#6A1B9A;}")
        send_btn.clicked.connect(self.apply_sync_forces)
        layout.addWidget(send_btn)
        
        group.setLayout(layout)
        return group
        
    def on_slider_changed(self, hand, axis, value):
        force_value = value / (self.force_resolution / 100.0)
        self.forces[hand][axis] = force_value
        (self.left_labels if hand == 'left' else self.right_labels)[axis].setText(f'{force_value:.2f} kg')
        self.publish_forces_if_changed()
        
    def _publish_force(self, hand, force):
        """发布单个手的力值（GUI 输入单位为 kg）"""
        self.force_ctrl.set_desired_ee_force_kg(
            hand,
            force_kg=(force['x'], force['y'], force['z']),
            torque=(0.0, 0.0, 0.0),
        )
        self.last_sent[hand] = force.copy()
    
    def publish_forces_if_changed(self, force_all=False):
        """只在力值发生变化时发送一次，force_all=True时强制发送"""
        if rospy.is_shutdown():
            return
        for hand in ['left', 'right']:
            if force_all or any(self.last_sent[hand][a] != self.forces[hand][a] for a in ['x', 'y', 'z']):
                self._publish_force(hand, self.forces[hand])
    
    def publish_forces(self):
        """强制发送一次"""
        self.publish_forces_if_changed(force_all=True)
        
    def reset_all_forces(self):
        self.zero_force('left')
        self.zero_force('right')
        # 清空同步输入区域的数值
        for axis in ['x', 'y', 'z']:
            self.sync_spinboxes[axis].setValue(0.0)
        
    def zero_left_force(self):
        self.zero_force('left')
        
    def zero_right_force(self):
        self.zero_force('right')
        
    def zero_force(self, hand):
        sliders = self.left_sliders if hand == 'left' else self.right_sliders
        labels = self.left_labels if hand == 'left' else self.right_labels
        for axis in ['x', 'y', 'z']:
            sliders[axis].blockSignals(True)
            sliders[axis].setValue(0)
            sliders[axis].blockSignals(False)
            self.forces[hand][axis] = 0.0
            labels[axis].setText('0.00 kg')
        self.publish_forces_if_changed()
    
    def apply_sync_forces(self):
        """将输入的力值同时应用到双手并发送"""
        force_values = {axis: self.sync_spinboxes[axis].value() for axis in ['x', 'y', 'z']}
        for hand in ['left', 'right']:
            sliders = self.left_sliders if hand == 'left' else self.right_sliders
            labels = self.left_labels if hand == 'left' else self.right_labels
            for axis in ['x', 'y', 'z']:
                slider_value = int(force_values[axis] * self.force_resolution / 100.0)
                sliders[axis].blockSignals(True)
                sliders[axis].setValue(slider_value)
                sliders[axis].blockSignals(False)
                self.forces[hand][axis] = force_values[axis]
                labels[axis].setText(f'{force_values[axis]:.2f} kg')
        self.publish_forces_if_changed()
    
    def toggle_force_empty_detact(self):
        """切换挥空检测开关状态并发送"""
        self.force_empty_detact_enabled = not self.force_empty_detact_enabled
        self.force_ctrl.enable_force_empty_detact(self.force_empty_detact_enabled)
        rospy.loginfo(f"Force empty detect: {'enabled' if self.force_empty_detact_enabled else 'disabled'}")
        # 更新按钮样式
        if self.force_empty_detact_enabled:
            text, color, hover, pressed = 'Disable Force Empty Detect', '#4CAF50', '#45a049', '#388E3C'
        else:
            text, color, hover, pressed = 'Enable Force Empty Detect', '#9E9E9E', '#757575', '#616161'
        self.empty_detact_btn.setText(text)
        self.empty_detact_btn.setStyleSheet(f"QPushButton{{background-color:{color};color:white;border:none;padding:12px 24px;border-radius:6px;font-size:12px;font-weight:bold;min-width:200px;min-height:40px;}}QPushButton:hover{{background-color:{hover};}}QPushButton:pressed{{background-color:{pressed};}}")
            
    def closeEvent(self, event):
        self.reset_all_forces()
        self.publish_forces()
        rospy.sleep(0.1)
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = EEForceControlGUI()
    window.show()
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
        window.publish_forces()
        window.publish_forces()
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
