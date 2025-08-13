import numpy as np
import pandas as pd
import time
import sys
import argparse
import termios
import tty
import select
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QComboBox, QPushButton, 
                           QSlider, QGroupBox, QGridLayout, QSpinBox, QDoubleSpinBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d, CubicSpline, PchipInterpolator

class NonBlockingInput:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        
    def __enter__(self):
        tty.setraw(sys.stdin.fileno())
        return self
        
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """非阻塞方式获取按键输入"""
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            return sys.stdin.read(1)
        return None

class SimpleMotionEditor(QWidget):
    def __init__(self, csv_path, dt=0.01, support_x_offset=0.0):
        super().__init__()
        self.csv_path = csv_path
        self.dt = dt
        self.support_x_offset = support_x_offset
        
        # 加载数据
        self.load_data()
        
        # 初始化历史记录
        self.history = []
        self.current_history_index = -1
        self.max_history_size = 2000
        
        # 设置matplotlib中文字体
        try:
            plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans', 'Arial Unicode MS']
        except:
            # 如果找不到中文字体，使用默认字体
            pass
        plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
        
        # 创建主布局
        main_layout = QVBoxLayout()
        main_layout.setSpacing(5)
        
        # 创建选择面板
        select_panel = QGroupBox("Trajectory Edit")
        select_layout = QHBoxLayout()
        select_layout.setSpacing(10)
        
        # 添加编辑模式选择
        mode_layout = QHBoxLayout()
        mode_layout.setSpacing(5)
        mode_layout.addWidget(QLabel('Edit Mode:'))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['Torso Edit', 'Left Leg Edit', 'Right Leg Edit', 'Contact Edit', 'Arm Edit'])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        select_layout.addLayout(mode_layout)
        
        # 添加维度选择
        dim_layout = QHBoxLayout()
        dim_layout.setSpacing(5)
        dim_layout.addWidget(QLabel('Edit Dimension:'))
        self.dim_combo = QComboBox()
        self.dim_combo.addItems(['X', 'Y', 'Z', 'Yaw'])  # 默认显示torso的维度选项
        self.dim_combo.currentIndexChanged.connect(self.on_dim_changed)
        dim_layout.addWidget(self.dim_combo)
        select_layout.addLayout(dim_layout)
        
        # 添加撤销和重做按钮
        undo_button = QPushButton("Undo (Ctrl+Z)")
        undo_button.clicked.connect(self.undo)
        select_layout.addWidget(undo_button)
        
        redo_button = QPushButton("Redo (Ctrl+Y)")
        redo_button.clicked.connect(self.redo)
        select_layout.addWidget(redo_button)
        
        # 添加保存按钮
        save_button = QPushButton("Save Trajectory")
        save_button.clicked.connect(self.save_trajectory)
        select_layout.addWidget(save_button)
        
        # 添加支撑点连线显示开关
        self.show_support_line_checkbox = QCheckBox("显示支撑点连线")
        self.show_support_line_checkbox.setChecked(False)
        self.show_support_line_checkbox.stateChanged.connect(self.on_support_line_toggled)
        select_layout.addWidget(self.show_support_line_checkbox)
        
        select_panel.setLayout(select_layout)
        main_layout.addWidget(select_panel)
        
        # 创建图表区域
        plot_layout = QVBoxLayout()
        plot_layout.setSpacing(5)
        
        # 创建主图表
        self.fig = plt.figure(figsize=(10, 8))
        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1])
        self.ax = self.fig.add_subplot(gs[0])
        self.contact_ax = self.fig.add_subplot(gs[1])
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setFocusPolicy(Qt.StrongFocus)
        self.canvas.setParent(self)
        self.canvas.installEventFilter(self)
        
        # 创建重置缩放按钮
        self.reset_zoom_button = QPushButton("Reset")
        self.reset_zoom_button.clicked.connect(self.reset_zoom)
        self.reset_zoom_button.setFixedWidth(60)
        self.reset_zoom_button.setFixedHeight(25)
        self.reset_zoom_button.setParent(self.canvas)
        self.reset_zoom_button.move(self.canvas.width() - 70, 10)
        self.reset_zoom_button.raise_()
        
        plot_layout.addWidget(self.canvas)
        
        # 创建时间轴
        timeline_layout = QHBoxLayout()
        timeline_layout.setSpacing(5)
        timeline_layout.addWidget(QLabel("时间轴:"))
        
        self.timeline_slider = QSlider(Qt.Horizontal)
        self.timeline_slider.setMinimum(0)
        self.timeline_slider.setMaximum(self.total_frames - 1)
        self.timeline_slider.setValue(0)
        self.timeline_slider.valueChanged.connect(self.on_timeline_changed)
        timeline_layout.addWidget(self.timeline_slider)
        
        self.time_label = QLabel("0.00s")
        timeline_layout.addWidget(self.time_label)
        
        plot_layout.addLayout(timeline_layout)
        main_layout.addLayout(plot_layout)
        
        # 创建控制面板
        control_panel = QGroupBox("控制")
        control_layout = QGridLayout()
        control_layout.setSpacing(5)
        
        # 插值点数输入
        interp_layout = QHBoxLayout()
        interp_layout.setSpacing(5)
        interp_layout.addWidget(QLabel("插值点数:"))
        
        self.interp_points_spin = QSpinBox()
        self.interp_points_spin.setMinimum(1)
        self.interp_points_spin.setMaximum(50000)
        self.interp_points_spin.setValue(5)
        interp_layout.addWidget(self.interp_points_spin)
        
        # 插值方式选择
        self.interp_method_combo = QComboBox()
        self.interp_method_combo.addItems(['三次样条', '线性'])
        interp_layout.addWidget(QLabel('插值方式:'))
        interp_layout.addWidget(self.interp_method_combo)
        
        control_layout.addLayout(interp_layout, 0, 0, 1, 2)
        
        # 播放控制
        play_layout = QHBoxLayout()
        play_layout.setSpacing(5)
        self.play_button = QPushButton("播放 (空格)")
        self.play_button.setCheckable(True)
        self.play_button.clicked.connect(self.toggle_play)
        play_layout.addWidget(self.play_button)
        
        self.prev_button = QPushButton("上一帧 (A)")
        self.prev_button.clicked.connect(self.prev_frame)
        play_layout.addWidget(self.prev_button)
        
        self.next_button = QPushButton("下一帧 (D)")
        self.next_button.clicked.connect(self.next_frame)
        play_layout.addWidget(self.next_button)
        
        control_layout.addLayout(play_layout, 1, 0, 1, 2)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(5)
        self.speed_label = QLabel("速度: 1.0x")
        speed_layout.addWidget(self.speed_label)
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(10)
        self.speed_slider.valueChanged.connect(self.update_speed)
        speed_layout.addWidget(self.speed_slider)
        
        control_layout.addLayout(speed_layout, 2, 0, 1, 2)
        
        control_panel.setLayout(control_layout)
        main_layout.addWidget(control_panel)
        
        # 设置布局
        self.setLayout(main_layout)
        
        # 连接事件
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        
        self.dragging = False
        self.selected_point = None
        self.edit_mode = 'torso'
        self.edit_axis = 0
        self.panning = False
        self.last_x = 0
        self.last_y = 0
        self.contact_dragging = False
        self.contact_selected_point = None
        self.show_support_line = False
        
        # 保存当前的缩放状态
        self.zoom_state = {
            'xlim': None,
            'ylim': None
        }
        
        # 记录修改过的帧
        self.modified_frames = set()
        
        # 计算抬腿相的最后一帧（接触相为1或2的区间）
        self.swing_end_frames_1, self.swing_end_frames_2 = self.find_swing_end_frames()
        
        # 初始化播放相关变量
        self._updating_timeline = False
        self.current_frame = 0
        self.playing = False
        self.playback_speed = 1.0
        self.last_update_time = time.time()
        self.accumulated_time = 0
        
        # 保存初始状态到历史记录
        self.save_to_history()
        
        # 初始化显示
        self.update_plot()
        self.canvas.draw()
        QApplication.processEvents()
        
        # 使用定时器延迟刷新布局
        QTimer.singleShot(100, self.refresh_layout)
    
    def load_data(self):
        """加载数据文件"""
        print(f"加载数据文件: {self.csv_path}")
        
        # 读取CSV文件
        self.df = pd.read_csv(self.csv_path, header=None)
        print(f"数据形状: {self.df.shape}")
        
        # 检查数据列数
        expected_cols = 1 + 1 + 4 + 3 + 3 + 8  # 时间 + 接触相 + torso + 左腿 + 右腿 + 手臂
        if len(self.df.columns) != expected_cols:
            print(f"警告: 期望{expected_cols}列，实际{len(self.df.columns)}列")
        
        self.total_frames = len(self.df)
        print(f"总帧数: {self.total_frames}")
        
        # 解析数据列
        col_idx = 0
        
        # 时间列
        self.time_data = self.df.iloc[:, col_idx].values
        col_idx += 1
        
        # 接触相列 (1列)
        self.contact_phases = self.df.iloc[:, col_idx].values
        col_idx += 1
        
        # torso列 (4列: x, y, z, yaw)
        self.torso_data = self.df.iloc[:, col_idx:col_idx+4].values
        col_idx += 4
        
        # 左腿列 (3列: x, y, yaw)
        self.left_leg_data = self.df.iloc[:, col_idx:col_idx+3].values
        col_idx += 3
        
        # 右腿列 (3列: x, y, yaw)
        self.right_leg_data = self.df.iloc[:, col_idx:col_idx+3].values
        col_idx += 3
        
        # 手臂列 (8列，可编辑)
        self.arm_data = self.df.iloc[:, col_idx:col_idx+8].values
        col_idx += 8
        
        print("数据加载完成")
        print(f"时间范围: {self.time_data[0]:.3f}s - {self.time_data[-1]:.3f}s")
        print(f"接触相范围: {np.min(self.contact_phases)} - {np.max(self.contact_phases)}")
        print(f"手臂数据形状: {self.arm_data.shape}")
        print(f"手臂数据范围: {np.min(self.arm_data):.3f} - {np.max(self.arm_data):.3f}")
    
    def find_swing_end_frames(self):
        """查找每个抬腿相（接触相为1或2）的最后一帧"""
        swing_end_frames_1 = []  # 接触相1的末尾帧
        swing_end_frames_2 = []  # 接触相2的末尾帧
        
        i = 0
        while i < self.total_frames:
            # 找到抬腿相的开始（接触相为1）
            while i < self.total_frames and self.contact_phases[i] != 1:
                i += 1
            if i >= self.total_frames:
                break
                
            start_frame = i
            # 找到抬腿相的结束
            while i < self.total_frames and self.contact_phases[i] == 1:
                i += 1
            end_frame = i - 1  # 最后一帧
            
            if end_frame >= start_frame:
                swing_end_frames_1.append(end_frame)
        
        # 重新开始查找接触相2
        i = 0
        while i < self.total_frames:
            # 找到抬腿相的开始（接触相为2）
            while i < self.total_frames and self.contact_phases[i] != 2:
                i += 1
            if i >= self.total_frames:
                break
                
            start_frame = i
            # 找到抬腿相的结束
            while i < self.total_frames and self.contact_phases[i] == 2:
                i += 1
            end_frame = i - 1  # 最后一帧
            
            if end_frame >= start_frame:
                swing_end_frames_2.append(end_frame)
        
        print(f"找到 {len(swing_end_frames_1)} 个接触相1结束帧: {swing_end_frames_1}")
        print(f"找到 {len(swing_end_frames_2)} 个接触相2结束帧: {swing_end_frames_2}")
        return swing_end_frames_1, swing_end_frames_2
    
    def reset_zoom(self):
        """重置缩放比例"""
        self.zoom_state['xlim'] = None
        self.zoom_state['ylim'] = None
        self.update_plot()
    
    def keyPressEvent(self, event):
        """处理键盘事件"""
        if event.key() == Qt.Key_Q:
            QApplication.quit()
        elif event.key() == Qt.Key_Space:
            self.playing = not self.playing
            self.play_button.setChecked(self.playing)
            if self.playing:
                self.last_update_time = time.time()
                self.accumulated_time = self.current_frame * self.dt
        elif event.key() == Qt.Key_D and not self.playing:
            self.display_frame(self.current_frame + 1)
        elif event.key() == Qt.Key_A and not self.playing:
            self.display_frame(self.current_frame - 1)
        elif event.key() == Qt.Key_W:
            self.playback_speed = min(self.playback_speed + 0.5, 10.0)
            self.speed_slider.setValue(int(self.playback_speed * 10))
            self.speed_label.setText(f"速度: {self.playback_speed:.1f}x")
        elif event.key() == Qt.Key_S:
            self.playback_speed = max(self.playback_speed - 0.5, 0.1)
            self.speed_slider.setValue(int(self.playback_speed * 10))
            self.speed_label.setText(f"速度: {self.playback_speed:.1f}x")
        elif event.key() == Qt.Key_R:
            self.current_frame = 0
            self.display_frame(0)
        elif event.key() == Qt.Key_Z and event.modifiers() == Qt.ControlModifier:
            self.undo()
        elif event.key() == Qt.Key_Y and event.modifiers() == Qt.ControlModifier:
            self.redo()
        else:
            super().keyPressEvent(event)
    
    def update_plot(self):
        """更新图表显示"""
        self.ax.clear()
        
        # 根据当前编辑模式选择数据
        if self.edit_mode == 'torso':
            data = self.torso_data
            title = f'Torso {self.dim_combo.currentText()}'
        elif self.edit_mode == 'left_leg':
            data = self.left_leg_data
            title = f'Left Leg {self.dim_combo.currentText()}'
        elif self.edit_mode == 'right_leg':
            data = self.right_leg_data
            title = f'Right Leg {self.dim_combo.currentText()}'
        elif self.edit_mode == 'arm':
            data = self.arm_data
            title = f'Arm {self.dim_combo.currentText()}'
        else:  # contact
            data = self.contact_phases
            title = f'Contact {self.dim_combo.currentText()}'
        
        # 绘制主曲线
        x = np.arange(self.total_frames)
        dim_idx = self.dim_combo.currentIndex()
        
        if self.edit_mode == 'contact':
            # 接触相数据只有一维
            y_data = data
            self.ax.plot(x, y_data, label=title, alpha=0.5)
            self.ax.scatter(x, y_data, c='blue', s=20, alpha=0.5)
        elif dim_idx < data.shape[1]:
            y_data = data[:, dim_idx]
            self.ax.plot(x, y_data, label=title, alpha=0.5)
            self.ax.scatter(x, y_data, c='blue', s=20, alpha=0.5)
        
        # 在torso编辑模式下显示支撑点连线
        if self.edit_mode == 'torso' and self.show_support_line:
            support_points = self.calculate_support_points()
            support_x = [point[0] for point in support_points]
            support_y = [point[1] for point in support_points]
            
            # 绘制支撑点连线
            if dim_idx == 0:  # X维度
                self.ax.plot(x, support_x, 'g--', label='Support Point X', alpha=0.7, linewidth=2)
            elif dim_idx == 1:  # Y维度
                self.ax.plot(x, support_y, 'g--', label='Support Point Y', alpha=0.7, linewidth=2)
            elif dim_idx == 2:  # Z维度
                # Z维度不显示支撑点，因为支撑点主要在XY平面
                pass
            elif dim_idx == 3:  # Yaw维度
                # 计算支撑点的朝向（基于双脚连线方向）
                support_yaw = []
                for frame_idx in range(self.total_frames):
                    left_foot = self.left_leg_data[frame_idx]
                    right_foot = self.right_leg_data[frame_idx]
                    # 计算双脚连线方向
                    dx = right_foot[0] - left_foot[0]
                    dy = right_foot[1] - left_foot[1]
                    yaw = np.arctan2(dy, dx)
                    support_yaw.append(yaw)
                self.ax.plot(x, support_yaw, 'g--', label='Support Point Yaw', alpha=0.7, linewidth=2)
        
        self.frame_line = self.ax.axvline(x=self.current_frame, color='r', alpha=0.5)
        
        # 高亮抬腿相的最后一帧
        for end_frame in self.swing_end_frames_1:
            self.ax.axvline(x=end_frame, color='red', alpha=0.7, linestyle='--', linewidth=2)
        for end_frame in self.swing_end_frames_2:
            self.ax.axvline(x=end_frame, color='blue', alpha=0.7, linestyle='--', linewidth=2)
        
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_title(title)
        
        # 清除并更新接触相图表
        self.contact_ax.clear()
        
        # 根据接触相值使用不同颜色
        x = np.arange(self.total_frames)
        colors = []
        for phase in self.contact_phases:
            if phase == 1:
                colors.append('red')
            elif phase == 2:
                colors.append('blue')
            else:
                colors.append('green')
        
        # 绘制接触相数据点
        for i in range(self.total_frames):
            self.contact_ax.scatter(i, self.contact_phases[i], c=colors[i], s=20, alpha=0.7)
        
        # 绘制连接线
        for i in range(self.total_frames - 1):
            self.contact_ax.plot([i, i+1], [self.contact_phases[i], self.contact_phases[i+1]], 
                               color=colors[i], alpha=0.5, linewidth=1)
        
        self.contact_frame_line = self.contact_ax.axvline(x=self.current_frame, color='r', alpha=0.5)
        
        self.contact_ax.grid(True)
        self.contact_ax.set_title('Contact Phase')
        self.contact_ax.set_ylim(-0.5, 3.5)
        
        # 恢复之前的缩放状态
        if self.zoom_state['xlim'] is not None:
            self.ax.set_xlim(self.zoom_state['xlim'])
            self.contact_ax.set_xlim(self.zoom_state['xlim'])
            if self.zoom_state['ylim'] is not None:
                self.ax.set_ylim(self.zoom_state['ylim'])
        
        # 调整图表布局
        self.fig.tight_layout()
        self.canvas.draw()
    
    def on_mode_changed(self, index):
        """处理编辑模式改变"""
        mode_names = ['torso', 'left_leg', 'right_leg', 'contact', 'arm']
        self.edit_mode = mode_names[index]
        
        # 根据编辑模式调整维度选择器
        self.dim_combo.clear()
        if self.edit_mode == 'torso':
            self.dim_combo.addItems(['X', 'Y', 'Z', 'Yaw'])  # torso有4个维度
        elif self.edit_mode in ['left_leg', 'right_leg']:
            self.dim_combo.addItems(['X', 'Y', 'Yaw'])  # 左右腿只有3个维度
        elif self.edit_mode == 'arm':
            self.dim_combo.addItems(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7', 'Joint 8'])  # 手臂有8个关节
        else:  # contact
            self.dim_combo.addItems(['Phase'])  # 接触相只有1个维度
        
        self.update_plot()
    
    def on_dim_changed(self, index):
        """处理维度选择改变"""
        self.edit_axis = index
        self.update_plot()
    
    def on_support_line_toggled(self, state):
        """处理支撑点连线显示切换"""
        self.show_support_line = state == Qt.Checked
        self.update_plot()
    
    def update_swing_phase_list(self):
        """更新摆动相列表"""
        self.swing_phase_combo.clear()
        self.swing_phase_combo.addItem("当前帧所在摆动相")
        
        # 找到所有摆动相
        swing_phases = self.find_all_swing_phases()
        
        for i, (phase, start_frame, end_frame) in enumerate(swing_phases):
            start_time = start_frame * self.dt
            end_time = end_frame * self.dt
            duration = (end_frame - start_frame + 1) * self.dt
            phase_name = f"摆动相{phase} ({start_time:.2f}s-{end_time:.2f}s, {duration:.2f}s)"
            self.swing_phase_combo.addItem(phase_name, (phase, start_frame, end_frame))
    
    def find_all_swing_phases(self):
        """找到所有摆动相"""
        swing_phases = []
        current_phase = None
        start_frame = 0
        
        for frame_idx in range(self.total_frames):
            phase = self.contact_phases[frame_idx]
            if phase in [1, 2]:  # 摆动相
                if current_phase != phase:
                    if current_phase is not None:
                        swing_phases.append((current_phase, start_frame, frame_idx - 1))
                    current_phase = phase
                    start_frame = frame_idx
            else:
                if current_phase is not None:
                    swing_phases.append((current_phase, start_frame, frame_idx - 1))
                    current_phase = None
        
        # 处理最后一个摆动相
        if current_phase is not None:
            swing_phases.append((current_phase, start_frame, self.total_frames - 1))
        
        return swing_phases
    
    def find_current_swing_phase(self):
        """找到当前帧所在的摆动相"""
        current_phase = self.contact_phases[self.current_frame]
        if current_phase not in [1, 2]:
            return None
        
        # 向前查找摆动相开始
        start_frame = self.current_frame
        while start_frame > 0 and self.contact_phases[start_frame - 1] == current_phase:
            start_frame -= 1
        
        # 向后查找摆动相结束
        end_frame = self.current_frame
        while end_frame < self.total_frames - 1 and self.contact_phases[end_frame + 1] == current_phase:
            end_frame += 1
        
        return current_phase, start_frame, end_frame
    
    def update_stance_phase_list(self):
        """更新站立相列表"""
        self.stance_phase_combo.clear()
        self.stance_phase_combo.addItem("当前帧所在站立相")
        
        # 找到所有站立相
        stance_phases = self.find_all_stance_phases()
        
        for i, (start_frame, end_frame) in enumerate(stance_phases):
            start_time = start_frame * self.dt
            end_time = end_frame * self.dt
            duration = (end_frame - start_frame + 1) * self.dt
            phase_name = f"站立相 ({start_time:.2f}s-{end_time:.2f}s, {duration:.2f}s)"
            self.stance_phase_combo.addItem(phase_name, (start_frame, end_frame))
    
    def find_all_stance_phases(self):
        """找到所有站立相（接触相为0）"""
        stance_phases = []
        current_phase = None
        start_frame = 0
        
        for frame_idx in range(self.total_frames):
            phase = self.contact_phases[frame_idx]
            if phase == 0:  # 站立相
                if current_phase != phase:
                    if current_phase is not None:
                        stance_phases.append((start_frame, frame_idx - 1))
                    current_phase = phase
                    start_frame = frame_idx
            else:
                if current_phase is not None:
                    stance_phases.append((start_frame, frame_idx - 1))
                    current_phase = None
        
        # 处理最后一个站立相
        if current_phase is not None:
            stance_phases.append((start_frame, self.total_frames - 1))
        
        return stance_phases
    
    def find_current_stance_phase(self):
        """找到当前帧所在的站立相"""
        current_phase = self.contact_phases[self.current_frame]
        if current_phase != 0:
            return None
        
        # 向前查找站立相开始
        start_frame = self.current_frame
        while start_frame > 0 and self.contact_phases[start_frame - 1] == 0:
            start_frame -= 1
        
        # 向后查找站立相结束
        end_frame = self.current_frame
        while end_frame < self.total_frames - 1 and self.contact_phases[end_frame + 1] == 0:
            end_frame += 1
        
        return start_frame, end_frame
    
    def apply_swing_time_adjustment(self):
        """应用摆动相时间调整"""
        time_adjustment = self.swing_time_spin.value()
        if abs(time_adjustment) < 0.01:  # 如果调整值很小，不执行
            return
        
        # 获取选中的摆动相
        selected_index = self.swing_phase_combo.currentIndex()
        if selected_index == 0:  # 当前帧所在摆动相
            swing_phase = self.find_current_swing_phase()
            if swing_phase is None:
                print("当前帧不在摆动相中")
                return
            phase, start_frame, end_frame = swing_phase
        else:
            # 从下拉列表获取选中的摆动相
            phase, start_frame, end_frame = self.swing_phase_combo.currentData()
        
        duration = end_frame - start_frame + 1
        target_duration = max(1, int(duration + time_adjustment * 10))  # 转换为帧数 (0.1s = 1帧)
        
        if target_duration != duration:
            self.adjust_swing_phase_duration(phase, start_frame, end_frame, target_duration)
            
            # 重新计算时间轴
            self.recalculate_time_axis()
            
            # 更新显示
            self.update_plot()
            self.save_to_history()
            
            print(f"摆动相 {phase} 时间调整完成，调整值: {time_adjustment}s")
            print(f"从 {duration} 帧调整到 {target_duration} 帧")
        else:
            print("调整值太小，无需调整")
    
    def apply_stance_time_adjustment(self):
        """应用站立相时间调整"""
        time_adjustment = self.stance_time_spin.value()
        if abs(time_adjustment) < 0.01:  # 如果调整值很小，不执行
            return
        
        # 获取选中的站立相
        selected_index = self.stance_phase_combo.currentIndex()
        if selected_index == 0:  # 当前帧所在站立相
            stance_phase = self.find_current_stance_phase()
            if stance_phase is None:
                print("当前帧不在站立相中")
                return
            start_frame, end_frame = stance_phase
        else:
            # 从下拉列表获取选中的站立相
            start_frame, end_frame = self.stance_phase_combo.currentData()
        
        duration = end_frame - start_frame + 1
        target_duration = max(1, int(duration + time_adjustment * 10))  # 转换为帧数 (0.1s = 1帧)
        
        if target_duration != duration:
            self.adjust_stance_phase_duration(start_frame, end_frame, target_duration)
            
            # 重新计算时间轴
            self.recalculate_time_axis()
            
            # 更新显示
            self.update_plot()
            self.save_to_history()
            
            print(f"站立相时间调整完成，调整值: {time_adjustment}s")
            print(f"从 {duration} 帧调整到 {target_duration} 帧")
        else:
            print("调整值太小，无需调整")
    
    def adjust_stance_phase_duration(self, start_frame, end_frame, target_duration):
        """调整单个站立相的持续时间"""
        current_duration = end_frame - start_frame + 1
        
        if target_duration > current_duration:
            # 延长站立相
            self.extend_stance_phase(start_frame, end_frame, target_duration)
        else:
            # 缩短站立相
            self.shorten_stance_phase(start_frame, end_frame, target_duration)
    
    def extend_stance_phase(self, start_frame, end_frame, target_duration):
        """延长站立相"""
        current_duration = end_frame - start_frame + 1
        extension = target_duration - current_duration
        
        # 在站立相结束后插入新的帧
        insert_position = end_frame + 1
        
        # 创建新帧的数据
        for i in range(extension):
            frame_idx = insert_position + i
            
            # 插入新帧到所有数据中
            self.time_data = np.insert(self.time_data, frame_idx, 
                                     self.time_data[end_frame] + (i + 1) * self.dt)
            self.contact_phases = np.insert(self.contact_phases, frame_idx, 0)  # 站立相为0
            self.torso_data = np.insert(self.torso_data, frame_idx, 
                                      self.torso_data[end_frame], axis=0)
            self.left_leg_data = np.insert(self.left_leg_data, frame_idx, 
                                         self.left_leg_data[end_frame], axis=0)
            self.right_leg_data = np.insert(self.right_leg_data, frame_idx, 
                                          self.right_leg_data[end_frame], axis=0)
            self.arm_data = np.insert(self.arm_data, frame_idx, 
                                    self.arm_data[end_frame], axis=0)
        
        # 更新总帧数
        self.total_frames += extension
        
        print(f"延长站立相，从 {current_duration} 帧到 {target_duration} 帧")
    
    def shorten_stance_phase(self, start_frame, end_frame, target_duration):
        """缩短站立相"""
        current_duration = end_frame - start_frame + 1
        reduction = current_duration - target_duration
        
        # 删除站立相末尾的帧
        for i in range(reduction):
            frame_idx = end_frame - i
            
            # 从所有数据中删除帧
            self.time_data = np.delete(self.time_data, frame_idx)
            self.contact_phases = np.delete(self.contact_phases, frame_idx)
            self.torso_data = np.delete(self.torso_data, frame_idx, axis=0)
            self.left_leg_data = np.delete(self.left_leg_data, frame_idx, axis=0)
            self.right_leg_data = np.delete(self.right_leg_data, frame_idx, axis=0)
            self.arm_data = np.delete(self.arm_data, frame_idx, axis=0)
        
        # 更新总帧数
        self.total_frames -= reduction
        
        print(f"缩短站立相，从 {current_duration} 帧到 {target_duration} 帧")
    
    def adjust_swing_phase_duration(self, phase, start_frame, end_frame, target_duration):
        """调整单个摆动相的持续时间"""
        current_duration = end_frame - start_frame + 1
        
        if target_duration > current_duration:
            # 延长摆动相
            self.extend_swing_phase(phase, start_frame, end_frame, target_duration)
        else:
            # 缩短摆动相
            self.shorten_swing_phase(phase, start_frame, end_frame, target_duration)
    
    def extend_swing_phase(self, phase, start_frame, end_frame, target_duration):
        """延长摆动相"""
        current_duration = end_frame - start_frame + 1
        extension = target_duration - current_duration
        
        # 在摆动相结束后插入新的帧
        insert_position = end_frame + 1
        
        # 创建新帧的数据
        for i in range(extension):
            frame_idx = insert_position + i
            
            # 插入新帧到所有数据中
            self.time_data = np.insert(self.time_data, frame_idx, 
                                     self.time_data[end_frame] + (i + 1) * self.dt)
            self.contact_phases = np.insert(self.contact_phases, frame_idx, phase)
            self.torso_data = np.insert(self.torso_data, frame_idx, 
                                      self.torso_data[end_frame], axis=0)
            self.left_leg_data = np.insert(self.left_leg_data, frame_idx, 
                                         self.left_leg_data[end_frame], axis=0)
            self.right_leg_data = np.insert(self.right_leg_data, frame_idx, 
                                          self.right_leg_data[end_frame], axis=0)
            self.arm_data = np.insert(self.arm_data, frame_idx, 
                                    self.arm_data[end_frame], axis=0)
        
        # 更新总帧数
        self.total_frames += extension
        
        print(f"延长摆动相 {phase}，从 {current_duration} 帧到 {target_duration} 帧")
    
    def shorten_swing_phase(self, phase, start_frame, end_frame, target_duration):
        """缩短摆动相"""
        current_duration = end_frame - start_frame + 1
        reduction = current_duration - target_duration
        
        # 删除摆动相末尾的帧
        for i in range(reduction):
            frame_idx = end_frame - i
            
            # 从所有数据中删除帧
            self.time_data = np.delete(self.time_data, frame_idx)
            self.contact_phases = np.delete(self.contact_phases, frame_idx)
            self.torso_data = np.delete(self.torso_data, frame_idx, axis=0)
            self.left_leg_data = np.delete(self.left_leg_data, frame_idx, axis=0)
            self.right_leg_data = np.delete(self.right_leg_data, frame_idx, axis=0)
            self.arm_data = np.delete(self.arm_data, frame_idx, axis=0)
        
        # 更新总帧数
        self.total_frames -= reduction
        
        print(f"缩短摆动相 {phase}，从 {current_duration} 帧到 {target_duration} 帧")
    
    def recalculate_time_axis(self):
        """重新计算时间轴"""
        # 重新计算时间数据
        for i in range(self.total_frames):
            self.time_data[i] = i * self.dt
        
        # 更新时间轴滑块
        self.timeline_slider.setMaximum(self.total_frames - 1)
        
        # 重新计算摆动相结束帧
        self.swing_end_frames_1, self.swing_end_frames_2 = self.find_swing_end_frames()
    
    def calculate_support_points(self):
        """计算每个时刻的支撑点位置"""
        support_points = []
        
        for frame_idx in range(self.total_frames):
            # 获取当前帧的接触相
            current_phase = self.contact_phases[frame_idx]
            
            # 根据接触相确定支撑脚
            if current_phase == 1:  # 左脚支撑
                support_foot = self.left_leg_data[frame_idx]
                support_x = support_foot[0] + self.support_x_offset  # X位置 + 偏置
                support_y = support_foot[1]  # Y位置
            elif current_phase == 2:  # 右脚支撑
                support_foot = self.right_leg_data[frame_idx]
                support_x = support_foot[0] + self.support_x_offset  # X位置 + 偏置
                support_y = support_foot[1]  # Y位置
            else:  # 双脚支撑或飞行相
                # 计算双脚中心点
                left_foot = self.left_leg_data[frame_idx]
                right_foot = self.right_leg_data[frame_idx]
                support_x = (left_foot[0] + right_foot[0]) / 2 + self.support_x_offset  # 中心点 + 偏置
                support_y = (left_foot[1] + right_foot[1]) / 2
            
            support_points.append((support_x, support_y))
        
        return support_points
    
    def on_timeline_changed(self, value):
        """处理时间轴改变"""
        if self._updating_timeline:
            return
        
        time = value * self.dt
        self.time_label.setText(f"{time:.2f}s")
        self.display_frame(value)
    
    def update_current_frame(self, frame_idx):
        """更新当前帧指示器"""
        self.current_frame = frame_idx
        self.frame_line.set_xdata([frame_idx, frame_idx])
        self.contact_frame_line.set_xdata([frame_idx, frame_idx])
        
        self._updating_timeline = True
        self.timeline_slider.setValue(frame_idx)
        self._updating_timeline = False
        
        time = frame_idx * self.dt
        self.time_label.setText(f"{time:.2f}s")
        self.canvas.draw()
    
    def toggle_play(self):
        """切换播放状态"""
        self.playing = not self.playing
        self.play_button.setChecked(self.playing)
        if self.playing:
            self.last_update_time = time.time()
            self.accumulated_time = self.current_frame * self.dt
    
    def prev_frame(self):
        """显示上一帧"""
        if not self.playing:
            self.display_frame(self.current_frame - 1)
    
    def next_frame(self):
        """显示下一帧"""
        if not self.playing:
            self.display_frame(self.current_frame + 1)
    
    def update_speed(self, value):
        """更新播放速度"""
        speed = value / 10.0
        self.playback_speed = speed
        self.speed_label.setText(f"速度: {speed:.1f}x")
    
    def display_frame(self, frame_idx):
        """显示指定帧"""
        if 0 <= frame_idx < self.total_frames:
            self.update_current_frame(frame_idx)
    
    def on_scroll(self, event):
        """处理滚轮事件"""
        if event.inaxes is None:
            return
        
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        
        axis_threshold = 20
        bbox = self.ax.get_window_extent()
        x, y = event.x, event.y
        
        is_near_xaxis = abs(y - bbox.y0) < axis_threshold
        is_near_yaxis = abs(x - bbox.x0) < axis_threshold
        
        base_scale = 0.9
        if event.button == 'down':
            base_scale = 1.1
        
        mouse_x = event.xdata
        mouse_y = event.ydata
        
        is_contact_ax = event.inaxes == self.contact_ax
        
        if is_contact_ax or (is_near_xaxis and not is_near_yaxis):
            x_range = xlim[1] - xlim[0]
            new_x_range = x_range * base_scale
            new_xlim = [
                mouse_x - (mouse_x - xlim[0]) * (new_x_range / x_range),
                mouse_x + (xlim[1] - mouse_x) * (new_x_range / x_range)
            ]
            self.ax.set_xlim(new_xlim)
            self.contact_ax.set_xlim(new_xlim)
        elif is_near_yaxis and not is_near_xaxis:
            y_range = ylim[1] - ylim[0]
            new_y_range = y_range * base_scale
            new_ylim = [
                mouse_y - (mouse_y - ylim[0]) * (new_y_range / y_range),
                mouse_y + (ylim[1] - mouse_y) * (new_y_range / y_range)
            ]
            self.ax.set_ylim(new_ylim)
        else:
            x_range = xlim[1] - xlim[0]
            y_range = ylim[1] - ylim[0]
            new_x_range = x_range * base_scale
            new_y_range = y_range * base_scale
            
            new_xlim = [
                mouse_x - (mouse_x - xlim[0]) * (new_x_range / x_range),
                mouse_x + (xlim[1] - mouse_x) * (new_x_range / x_range)
            ]
            new_ylim = [
                mouse_y - (mouse_y - ylim[0]) * (new_y_range / y_range),
                mouse_y + (ylim[1] - mouse_y) * (new_y_range / y_range)
            ]
            
            self.ax.set_xlim(new_xlim)
            self.ax.set_ylim(new_ylim)
            self.contact_ax.set_xlim(new_xlim)
        
        self.zoom_state['xlim'] = self.ax.get_xlim()
        self.zoom_state['ylim'] = self.ax.get_ylim()
        
        self.canvas.draw()
    
    def on_click(self, event):
        """处理鼠标点击事件"""
        if event.inaxes is None:
            return
        
        if event.button == 2:  # 中键
            self.panning = True
            self.last_x = event.xdata
            self.last_y = event.ydata
            return
        
        # 检查是否点击在接触相图表上
        if event.inaxes == self.contact_ax:
            new_value = round(event.ydata)
            new_value = max(0, min(3, new_value))
            
            frame_idx = int(round(event.xdata))
            if 0 <= frame_idx < self.total_frames:
                self.contact_dragging = True
                self.contact_start_frame = frame_idx
                self.contact_start_value = new_value
                
                # 更新接触相数据
                self.contact_phases[frame_idx] = new_value
                
                self.display_frame(frame_idx)
                self.update_plot()
            return
        
        # 主图表的点击处理
        x_data = np.arange(self.total_frames)
        
        if self.edit_mode == 'torso':
            y_data = self.torso_data[:, self.edit_axis]
        elif self.edit_mode == 'left_leg':
            y_data = self.left_leg_data[:, self.edit_axis]
        elif self.edit_mode == 'right_leg':
            y_data = self.right_leg_data[:, self.edit_axis]
        elif self.edit_mode == 'arm':
            y_data = self.arm_data[:, self.edit_axis]
        else:  # contact
            y_data = self.contact_phases  # 接触相数据是一维的
        
        points_display = np.column_stack([x_data, y_data])
        display_coords = self.ax.transData.transform(points_display)
        click_display = self.ax.transData.transform((event.xdata, event.ydata))
        
        distances = np.sqrt(np.sum((display_coords - click_display) ** 2, axis=1))
        nearest_idx = np.argmin(distances)
        
        pixel_threshold = 10
        
        if distances[nearest_idx] < pixel_threshold:
            self.dragging = True
            self.selected_point = nearest_idx
    
    def on_motion(self, event):
        """处理鼠标移动事件"""
        if event.inaxes is None:
            return
        
        if self.panning:
            dx = event.xdata - self.last_x
            dy = event.ydata - self.last_y
            
            self.ax.set_xlim(self.ax.get_xlim()[0] - dx, self.ax.get_xlim()[1] - dx)
            self.ax.set_ylim(self.ax.get_ylim()[0] - dy, self.ax.get_ylim()[1] - dy)
            self.contact_ax.set_xlim(self.contact_ax.get_xlim()[0] - dx, self.contact_ax.get_xlim()[1] - dx)
            
            self.last_x = event.xdata
            self.last_y = event.ydata
            
            self.zoom_state['xlim'] = self.ax.get_xlim()
            self.zoom_state['ylim'] = self.ax.get_ylim()
            
            self.canvas.draw()
            return
        
        if self.contact_dragging:
            new_value = round(event.ydata)
            new_value = max(0, min(3, new_value))
            
            current_frame = int(round(event.xdata))
            if 0 <= current_frame < self.total_frames:
                start_frame = min(self.contact_start_frame, current_frame)
                end_frame = max(self.contact_start_frame, current_frame)
                
                for frame_idx in range(start_frame, end_frame + 1):
                    if 0 <= frame_idx < self.total_frames:
                        self.contact_phases[frame_idx] = new_value
                
                self.display_frame(current_frame)
                self.update_plot()
                self.save_to_history()
            return
        
        if not self.dragging:
            return
        
        # 更新选中点的值
        if self.edit_mode == 'torso':
            self.torso_data[self.selected_point, self.edit_axis] = event.ydata
        elif self.edit_mode == 'left_leg':
            self.left_leg_data[self.selected_point, self.edit_axis] = event.ydata
        elif self.edit_mode == 'right_leg':
            self.right_leg_data[self.selected_point, self.edit_axis] = event.ydata
        elif self.edit_mode == 'arm':
            self.arm_data[self.selected_point, self.edit_axis] = event.ydata
        else:  # contact
            self.contact_phases[self.selected_point] = event.ydata  # 接触相数据是一维的
        
        if self.selected_point is not None:
            self.modified_frames.add(int(self.selected_point))
        self.interpolate_values()
        self.update_plot()
        self.save_to_history()
    
    def on_release(self, event):
        """处理鼠标释放事件"""
        if event.button == 2:  # 中键
            self.panning = False
        else:
            if self.dragging or self.contact_dragging:
                self.save_to_history()
            self.dragging = False
            self.selected_point = None
            self.contact_dragging = False
            self.contact_selected_point = None
    
    def interpolate_values(self):
        """对修改的值进行插值"""
        if self.selected_point is None:
            return
            
        range_points = self.interp_points_spin.value()
        is_cubic = self.interp_method_combo.currentText() == '三次样条'
        
        if self.edit_mode == 'torso':
            data = self.torso_data
            axis_data = data[:, self.edit_axis]
        elif self.edit_mode == 'left_leg':
            data = self.left_leg_data
            axis_data = data[:, self.edit_axis]
        elif self.edit_mode == 'right_leg':
            data = self.right_leg_data
            axis_data = data[:, self.edit_axis]
        elif self.edit_mode == 'arm':
            data = self.arm_data
            axis_data = data[:, self.edit_axis]
        else:  # contact
            data = self.contact_phases
            axis_data = data  # 接触相数据是一维的
        x = np.arange(self.total_frames)
        
        start_idx = max(0, self.selected_point - range_points)
        end_idx = min(self.total_frames, self.selected_point + range_points + 1)
        
        if end_idx - start_idx > 1:
            x_interp = np.arange(start_idx, end_idx)
            
            if is_cubic and end_idx - start_idx > 3:
                x_points = np.array([start_idx, self.selected_point, end_idx - 1])
                y_points = np.array([axis_data[start_idx], axis_data[self.selected_point], axis_data[end_idx - 1]])
                
                f = interp1d(x_points, y_points, kind='quadratic', bounds_error=False, fill_value='extrapolate')
                axis_data[start_idx:end_idx] = f(x_interp)
            else:
                axis_data[start_idx:end_idx] = np.interp(
                    x_interp,
                    np.array([start_idx, self.selected_point, end_idx - 1]),
                    np.array([axis_data[start_idx], axis_data[self.selected_point], axis_data[end_idx - 1]])
                )
            
            self.modified_frames.update(range(start_idx, end_idx))
        
        # 将插值结果赋值回原数据
        if self.edit_mode == 'contact':
            data[:] = axis_data  # 接触相数据是一维的
        else:
            data[:, self.edit_axis] = axis_data
    
    def resizeEvent(self, event):
        """处理窗口大小改变事件"""
        super().resizeEvent(event)
        if hasattr(self, 'reset_zoom_button'):
            self.reset_zoom_button.move(self.canvas.width() - 70, 10)
    
    def save_to_history(self):
        """保存当前状态到历史记录"""
        current_state = {
            'torso_data': self.torso_data.copy(),
            'left_leg_data': self.left_leg_data.copy(),
            'right_leg_data': self.right_leg_data.copy(),
            'arm_data': self.arm_data.copy(),
            'contact_phases': self.contact_phases.copy(),
            'modified_frames': self.modified_frames.copy()
        }
        
        # 删除当前历史记录之后的所有记录
        if self.current_history_index < len(self.history) - 1:
            self.history = self.history[:self.current_history_index + 1]
        
        # 添加新的历史记录
        self.history.append(current_state)
        self.current_history_index = len(self.history) - 1
        
        # 如果历史记录超过最大数量，删除最早的记录
        if len(self.history) > self.max_history_size:
            self.history.pop(0)
            self.current_history_index -= 1
    
    def undo(self):
        """撤销操作"""
        if self.current_history_index > 0:
            self.current_history_index -= 1
            self.restore_state(self.history[self.current_history_index])
            self.update_plot()
    
    def redo(self):
        """重做操作"""
        if self.current_history_index < len(self.history) - 1:
            self.current_history_index += 1
            self.restore_state(self.history[self.current_history_index])
            self.update_plot()
    
    def restore_state(self, state):
        """恢复指定状态"""
        self.torso_data = state['torso_data'].copy()
        self.left_leg_data = state['left_leg_data'].copy()
        self.right_leg_data = state['right_leg_data'].copy()
        self.arm_data = state['arm_data'].copy()
        self.contact_phases = state['contact_phases'].copy()
        self.modified_frames = state['modified_frames'].copy()
    

    
    def save_trajectory(self):
        """保存修改后的轨迹到CSV文件"""
        # 创建结果DataFrame
        results_data = []
        
        for frame_idx in range(self.total_frames):
            # 时间
            row = [self.time_data[frame_idx]]
            
            # 接触相
            row.append(self.contact_phases[frame_idx])
            
            # torso
            row.extend(self.torso_data[frame_idx])
            
            # 左腿
            row.extend(self.left_leg_data[frame_idx])
            
            # 右腿
            row.extend(self.right_leg_data[frame_idx])
            
            # 手臂（保持不变）
            row.extend(self.arm_data[frame_idx])
            
            results_data.append(row)
        
        # 创建DataFrame并保存
        results_df = pd.DataFrame(results_data)
        
        # 生成输出文件名
        file_name, file_ext = os.path.splitext(self.csv_path)
        output_file = f"{file_name}{file_ext}"
        
        results_df.to_csv(output_file, index=False, header=False)
        
        print(f"轨迹已保存到 {output_file}")
        print(f"共修改了 {len(self.modified_frames)} 帧")
    
    def eventFilter(self, obj, event):
        """事件过滤器，用于处理键盘事件"""
        if obj == self.canvas and event.type() == event.KeyPress:
            self.keyPressEvent(event)
            return True
        return super().eventFilter(obj, event)
    
    def refresh_layout(self):
        """刷新布局"""
        self.update_plot()
        self.canvas.draw()
        QApplication.processEvents()

def main():
    parser = argparse.ArgumentParser(description='单步运动编辑器')
    parser.add_argument('--csv', type=str, 
                      default='/root/Workspace/kuavo-ros-control/src/demo/csv2body_demo/actions/taiji_step_roban_stable.csv',
                      help='数据文件路径')
    parser.add_argument('--dt', type=float, default=0.01,
                      help='时间间隔')
    
    args = parser.parse_args()
    
    # 在主函数中直接定义支撑点X方向偏置
    support_x_offset = 0.0  # 可以根据需要调整这个值
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    
    # 创建编辑器
    editor = SimpleMotionEditor(args.csv, args.dt, support_x_offset)
    editor.show()
    
    # 运行Qt事件循环
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 