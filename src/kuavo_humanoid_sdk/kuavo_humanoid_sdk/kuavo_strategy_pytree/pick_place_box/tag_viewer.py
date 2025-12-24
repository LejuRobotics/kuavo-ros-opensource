#!/usr/bin/env python3
"""AprilTag Position Viewer - Compressed"""
import sys
import rospy
import numpy as np
from kuavo_msgs.msg import AprilTagDetectionArray
from collections import deque
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class TagViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AprilTag Position Viewer")
        self.setGeometry(100, 100, 1200, 800)
        
        self.tag_data_history = {}  
        self.max_history_length = 3000
        self.start_time = None
        self.time_window_seconds = 30.0
        self.current_time_offset = 0.0
        self.is_live_mode = True
        self.xyz_colors = {'x': '#FF0000', 'y': '#00AA00', 'z': '#0000FF'}
        self.position_axes = {}
        
        rospy.init_node('tag_viewer', anonymous=True)
        self.tag_subscriber = rospy.Subscriber('/detected_tags', AprilTagDetectionArray, self.tag_callback, queue_size=10)
        
        self.setup_ui()
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(100)
        
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        title_label = QLabel("AprilTag Position Curves")
        title_font = QFont()
        title_font.setPointSize(14), title_font.setBold(True)
        title_label.setFont(title_font), title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        self.position_figure = Figure(figsize=(12, 8))
        self.position_canvas = FigureCanvas(self.position_figure)
        main_layout.addWidget(self.position_canvas)
        
        # Time control
        time_control_widget = QWidget()
        time_control_layout = QVBoxLayout(time_control_widget)
        slider_layout = QHBoxLayout()
        
        self.time_label = QLabel("Time Control:")
        self.time_label.setFont(QFont("Arial", 10, QFont.Bold)), slider_layout.addWidget(self.time_label)
        
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_slider.setMinimum(0)
        self.time_slider.setMaximum(100)
        self.time_slider.setValue(100)
        self.time_slider.setTickPosition(QSlider.TicksBelow)
        self.time_slider.setTickInterval(10)
        self.time_slider.setMinimumHeight(40)
        self.time_slider.setStyleSheet("""
            QSlider::groove:horizontal { height: 12px; background: #E0E0E0; border-radius: 6px; }
            QSlider::handle:horizontal { background: #4CAF50; border: 2px solid #FFF; width: 26px; height: 26px; margin: -8px 0; border-radius: 13px; }
            QSlider::handle:horizontal:hover { background: #5CBF60; }
            QSlider::sub-page:horizontal { background: #4CAF50; border-radius: 6px; }
        """)
        self.time_slider.valueChanged.connect(self.on_time_slider_changed)
        slider_layout.addWidget(self.time_slider, 1)
        
        self.time_start_label = QLabel("0s")
        self.time_current_label = QLabel("Live")
        self.time_end_label = QLabel("30s")
        
        label_style = "font-weight: bold; font-size: 11px; padding: 2px 8px;"
        self.time_start_label.setStyleSheet(label_style + "color: #666;")
        self.time_current_label.setStyleSheet(label_style + "color: #4CAF50;")
        self.time_end_label.setStyleSheet(label_style + "color: #666;")
        
        slider_layout.addWidget(self.time_start_label)
        slider_layout.addWidget(self.time_current_label)
        slider_layout.addWidget(self.time_end_label)
        
        time_control_layout.addLayout(slider_layout)
        instruction_label = QLabel("Drag slider to view historical data • Right end = Live mode")
        instruction_label.setStyleSheet("color: #888; font-size: 10px; font-style: italic;"), instruction_label.setAlignment(Qt.AlignCenter)
        time_control_layout.addWidget(instruction_label)
        
        main_layout.addWidget(time_control_widget)
        
        self.status_label = QLabel("Waiting for data...")
        self.status_label.setAlignment(Qt.AlignCenter), main_layout.addWidget(self.status_label)
        
    def tag_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.to_sec()
        
        current_time = msg.header.stamp.to_sec() - self.start_time
        
        for detection in msg.detections:
            if len(detection.id) > 0:
                tag_id = detection.id[0]
                
                if tag_id not in self.tag_data_history:
                    self.tag_data_history[tag_id] = {
                        'time': deque(maxlen=self.max_history_length),
                        'x': deque(maxlen=self.max_history_length),
                        'y': deque(maxlen=self.max_history_length),
                        'z': deque(maxlen=self.max_history_length)
                    }
                
                pos = detection.pose.pose.pose.position
                data = self.tag_data_history[tag_id]
                data['time'].append(current_time)
                data['x'].append(pos.x)
                data['y'].append(pos.y)
                data['z'].append(pos.z)
                
    def on_time_slider_changed(self, value):
        if len(self.tag_data_history) == 0:
            return
            
        max_time = 0
        for tag_data in self.tag_data_history.values():
            if len(tag_data['time']) > 0:
                max_time = max(max_time, max(tag_data['time']))
        
        if max_time == 0:
            return
            
        self.is_live_mode = (value == self.time_slider.maximum())
        
        if self.is_live_mode:
            self.current_time_offset = max_time
            self.time_current_label.setText("Live")
        else:
            slider_ratio = value / self.time_slider.maximum()
            self.current_time_offset = max_time * slider_ratio
            self.time_current_label.setText(f"{self.current_time_offset:.1f}s")
        
        self.time_end_label.setText(f"{max_time:.1f}s")
        
    def get_time_window_data(self, tag_data):
        if len(tag_data['time']) == 0:
            return {}, {}
            
        time_array = np.array(tag_data['time'])
        
        if self.is_live_mode:
            end_time = time_array[-1]
            start_time = max(0, end_time - self.time_window_seconds)
        else:
            center_time = self.current_time_offset
            start_time = max(0, center_time - self.time_window_seconds / 2)
            end_time = center_time + self.time_window_seconds / 2
        
        mask = (time_array >= start_time) & (time_array <= end_time)
        indices = np.where(mask)[0]
        
        if len(indices) == 0:
            return {}, {}
            
        windowed_data = {}
        for key in ['time', 'x', 'y', 'z']:
            data_array = np.array(tag_data[key])
            windowed_data[key] = data_array[indices]
            
        current_point = {}
        if not self.is_live_mode:
            time_diffs = np.abs(time_array - self.current_time_offset)
            closest_idx = np.argmin(time_diffs)
            if time_diffs[closest_idx] < 1.0:
                for key in ['time', 'x', 'y', 'z']:
                    data_array = np.array(tag_data[key])
                    current_point[key] = data_array[closest_idx]
                    
        return windowed_data, current_point
    
    def create_subplots_for_tags(self):
        tag_ids = list(self.tag_data_history.keys())
        num_tags = len(tag_ids)
        
        if num_tags == 0:
            return
            
        self.position_figure.clear()
        self.position_axes = {}
        
        for i, tag_id in enumerate(sorted(tag_ids)):
            ax = self.position_figure.add_subplot(num_tags, 1, i + 1)
            self.position_axes[tag_id] = ax
            ax.grid(True, alpha=0.3)
            ax.set_ylabel('Position (m)')
            ax.set_title(f'Tag {tag_id} Position vs Time (30s Window)')
            if i == num_tags - 1:
                ax.set_xlabel('Time (s)')
                
        self.position_figure.tight_layout(pad=1.5)
        
    def update_plots(self):
        if len(self.tag_data_history) == 0:
            self.status_label.setText("Waiting for data...")
            return
        
        self.status_label.setText(f"Displaying {len(self.tag_data_history)} tag(s)")
        
        max_time = 0
        for tag_data in self.tag_data_history.values():
            if len(tag_data['time']) > 0:
                max_time = max(max_time, max(tag_data['time']))
                
        if max_time > 0:
            self.time_slider.setMaximum(int(max_time * 10))
            if self.is_live_mode:
                self.time_slider.setValue(self.time_slider.maximum())
        
        self.update_position_plot()
        
    def update_position_plot(self):
        current_tags = set(self.tag_data_history.keys())
        existing_tags = set(self.position_axes.keys())
        
        if current_tags != existing_tags:
            self.create_subplots_for_tags()
        
        for tag_id, tag_data in self.tag_data_history.items():
            if tag_id not in self.position_axes:
                continue
                
            ax = self.position_axes[tag_id]
            ax.clear()
            
            windowed_data, current_point = self.get_time_window_data(tag_data)
            
            if len(windowed_data) == 0:
                ax.grid(True, alpha=0.3)
                ax.set_ylabel('Position (m)')
                ax.set_title(f'Tag {tag_id} Position vs Time (30s Window)')
                continue
            
            if len(windowed_data['time']) > 0:
                ax.plot(windowed_data['time'], windowed_data['x'], color=self.xyz_colors['x'], linewidth=2.5, label='X Position', alpha=0.9)
                ax.plot(windowed_data['time'], windowed_data['y'], color=self.xyz_colors['y'], linewidth=2.5, label='Y Position', alpha=0.9)
                ax.plot(windowed_data['time'], windowed_data['z'], color=self.xyz_colors['z'], linewidth=2.5, label='Z Position', alpha=0.9)
                
                if self.is_live_mode and len(windowed_data['time']) > 0:
                    current_x, current_y, current_z = windowed_data['x'][-1], windowed_data['y'][-1], windowed_data['z'][-1]
                    current_time = windowed_data['time'][-1]
                elif current_point:
                    current_x, current_y, current_z = current_point['x'], current_point['y'], current_point['z']
                    current_time = current_point['time']
                    ax.plot(current_point['time'], current_point['x'], 'o', color=self.xyz_colors['x'], markersize=10, alpha=0.8)
                    ax.plot(current_point['time'], current_point['y'], 'o', color=self.xyz_colors['y'], markersize=10, alpha=0.8)
                    ax.plot(current_point['time'], current_point['z'], 'o', color=self.xyz_colors['z'], markersize=10, alpha=0.8)
                else:
                    current_x = windowed_data['x'][-1] if len(windowed_data['x']) > 0 else 0.0
                    current_y = windowed_data['y'][-1] if len(windowed_data['y']) > 0 else 0.0
                    current_z = windowed_data['z'][-1] if len(windowed_data['z']) > 0 else 0.0
                    current_time = windowed_data['time'][-1] if len(windowed_data['time']) > 0 else 0.0
                
                data_text = f"X: {current_x:.2f}m  Y: {current_y:.2f}m  Z: {current_z:.2f}m  t: {current_time:.2f}s"
                ax.text(0.02, 0.98, data_text, transform=ax.transAxes, fontsize=10, fontweight='bold', 
                       verticalalignment='top', horizontalalignment='left',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8, edgecolor='gray'))
            
            ax.set_ylabel('Position (m)')
            ax.set_title(f'Tag {tag_id} Position vs Time (30s Window)')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize=9)
            
            if tag_id == max(self.tag_data_history.keys()):
                ax.set_xlabel('Time (s)')
        
        self.position_figure.tight_layout(pad=1.5)
        self.position_canvas.draw()
    
    def closeEvent(self, event):
        self.update_timer.stop()
        event.accept()

def main():
    try:
        app = QApplication(sys.argv)
        viewer = TagViewer()
        viewer.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()