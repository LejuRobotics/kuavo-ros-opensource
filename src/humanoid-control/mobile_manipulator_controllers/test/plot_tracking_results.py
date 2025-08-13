#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_tracking_results(csv_folder="/tmp"):
    """
    Read MPC trajectory tracking test CSV files and plot comparison curves
    
    Args:
        csv_folder: CSV files folder path
    """
    
    # File name list
    file_names = [
        "tracking_test_single_frame.csv",
        "tracking_test_two_frame_window.csv", 
        "tracking_test_all_frames.csv",
        "tracking_test_three_frame_prediction.csv"
    ]
    
    # Target trajectory file names
    target_file_names = [
        "tracking_test_single_frame_target.csv",
        "tracking_test_two_frame_window_target.csv", 
        "tracking_test_all_frames_target.csv",
        "tracking_test_three_frame_prediction_target.csv"
    ]
    
    predefined_eef_target_file_name = "predefined_eef_target.csv"
    
    # Mode names
    mode_names = ["Single Frame", "Two Frame Window", "All Frames", "Three Frame Prediction"]
    
    # Color settings
    colors = ['red', 'blue', 'green', 'cyan']
    target_colors = ['darkred', 'darkblue', 'darkgreen', 'darkcyan']
    
    # Read data
    data_dict = {}
    target_dict = {}
    predefined_eef_target_dict = None
    filepath = os.path.join(csv_folder, predefined_eef_target_file_name)
    print(f"predefined_eef_target_file_name filepath: {filepath}")
    if os.path.exists(filepath):
        predefined_eef_target_dict = pd.read_csv(filepath)
        print(f"Successfully loaded: {predefined_eef_target_file_name}")
    else:
        print(f"File not found: {filepath}")
        return
    
    for i, filename in enumerate(file_names):
        filepath = os.path.join(csv_folder, filename)
        target_filepath = os.path.join(csv_folder, target_file_names[i])
        
        if os.path.exists(filepath):
            data_dict[mode_names[i]] = pd.read_csv(filepath)
            print(f"Successfully loaded: {filename}")
        else:
            print(f"File not found: {filepath}")
            return
            
        if os.path.exists(target_filepath):
            target_dict[mode_names[i]] = pd.read_csv(target_filepath)
            print(f"Successfully loaded target: {target_file_names[i]}")
        else:
            print(f"Target file not found: {target_filepath}")
            target_dict[mode_names[i]] = None
    
    
    # Set font
    # plt.rcParams['font.family'] = 'Arial'
    
    # Create figure
    fig = plt.figure(figsize=(20, 15))
    
    # 1. Base position comparison (x, y, z)
    for i, axis in enumerate(['x', 'y', 'z']):
        plt.subplot(4, 3, i+1)
        for j, (mode_name, data) in enumerate(data_dict.items()):
            plt.plot(data['time'], data[f'state_{i}'], 
                    color=colors[j], label=mode_name, linewidth=2)
            # Plot target trajectory if available
            # if target_dict[mode_name] is not None:
            #     plt.plot(target_dict[mode_name]['time'], target_dict[mode_name][f'target_state_{i}'], 
            #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
        plt.xlabel('Time (s)')
        plt.ylabel(f'Base Position {axis} (m)')
        plt.title(f'Base {axis}-axis Position Tracking Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    # 2. Base attitude comparison (roll, pitch, yaw)
    attitude_names = ['Roll', 'Pitch', 'Yaw']
    for i, attitude in enumerate(attitude_names):
        plt.subplot(4, 3, i+4)
        for j, (mode_name, data) in enumerate(data_dict.items()):
            plt.plot(data['time'], data[f'state_{i+3}'] * 180/np.pi, 
                    color=colors[j], label=mode_name, linewidth=2)
            # Plot target trajectory if available
            # if target_dict[mode_name] is not None:
            #     plt.plot(target_dict[mode_name]['time'], target_dict[mode_name][f'target_state_{i+3}'] * 180/np.pi, 
            #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
        plt.xlabel('Time (s)')
        plt.ylabel(f'{attitude} (deg)')
        plt.title(f'Base {attitude} Angle Tracking Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    # 3. Left arm joint angle comparison (first 3 joints)
    for i in range(3):
        plt.subplot(4, 3, i+7)
        for j, (mode_name, data) in enumerate(data_dict.items()):
            plt.plot(data['time'], data[f'state_{i+6}'] * 180/np.pi, 
                    color=colors[j], label=mode_name, linewidth=2)
            # Plot target trajectory if available
            # if target_dict[mode_name] is not None:
            #     plt.plot(target_dict[mode_name]['time'], target_dict[mode_name][f'target_state_{i+6}'] * 180/np.pi, 
            #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
        plt.xlabel('Time (s)')
        plt.ylabel(f'Left Arm Joint {i+1} (deg)')
        plt.title(f'Left Arm Joint {i+1} Angle Tracking Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    # 4. Right arm joint angle comparison (first 3 joints)
    for i in range(3):
        plt.subplot(4, 3, i+10)
        for j, (mode_name, data) in enumerate(data_dict.items()):
            plt.plot(data['time'], data[f'state_{i+13}'] * 180/np.pi, 
                    color=colors[j], label=mode_name, linewidth=2)
            # Plot target trajectory if available
            if target_dict[mode_name] is not None:
                plt.plot(target_dict[mode_name]['time'], target_dict[mode_name][f'target_state_{i+13}'] * 180/np.pi, 
                        color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
        plt.xlabel('Time (s)')
        plt.ylabel(f'Right Arm Joint {i+1} (deg)')
        plt.title(f'Right Arm Joint {i+1} Angle Tracking Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(csv_folder, 'joint_tracking_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"Joint tracking comparison plot saved: {os.path.join(csv_folder, 'joint_tracking_comparison.png')}")
    
    # Create end-effector position comparison plot
    fig2 = plt.figure(figsize=(15, 10))
    
    # Left hand end-effector position (x, y, z)
    for i, axis in enumerate(['x', 'y', 'z']):
        plt.subplot(2, 3, i+1)
        for j, (mode_name, data) in enumerate(data_dict.items()):
            plt.plot(data['time'], data[f'eef_{i}'], 
                    color=colors[j], label=mode_name, linewidth=2)
            # Plot target trajectory if available
            # if target_dict[mode_name] is not None:
            #     plt.plot(target_dict[mode_name]['time'], target_dict[mode_name][f'target_eef_{i}'], 
            #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
        if predefined_eef_target_dict is not None:
            plt.plot(predefined_eef_target_dict['time'], predefined_eef_target_dict[f'eef_target_{i}'], 
                    color='purple', linestyle='--', alpha=0.7, linewidth=0.5, label='Predefined EEF Target')
        plt.xlabel('Time (s)')
        plt.ylabel(f'Left Hand Position {axis} (m)')
        plt.title(f'Left Hand {axis}-axis Position Tracking Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    # Right hand end-effector position (x, y, z)
    for i, axis in enumerate(['x', 'y', 'z']):
        plt.subplot(2, 3, i+4)
        for j, (mode_name, data) in enumerate(data_dict.items()):
            plt.plot(data['time'], data[f'eef_{i+7}'], 
                    color=colors[j], label=mode_name, linewidth=2)
            # Plot target trajectory if available
            # if target_dict[mode_name] is not None:
            #     plt.plot(target_dict[mode_name]['time'], target_dict[mode_name][f'target_eef_{i+7}'], 
            #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
        if predefined_eef_target_dict is not None:
            plt.plot(predefined_eef_target_dict['time'], predefined_eef_target_dict[f'eef_target_{i+7}'], 
                    color='purple', linestyle='--', alpha=0.7, linewidth=0.5, label='Predefined EEF Target')
        plt.xlabel('Time (s)')
        plt.ylabel(f'Right Hand Position {axis} (m)')
        plt.title(f'Right Hand {axis}-axis Position Tracking Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(csv_folder, 'eef_position_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"End-effector position comparison plot saved: {os.path.join(csv_folder, 'eef_position_comparison.png')}")
    
    # Create 3D trajectory comparison plot
    fig3 = plt.figure(figsize=(15, 5))
    
    # Left hand 3D trajectory
    ax1 = fig3.add_subplot(121, projection='3d')
    for j, (mode_name, data) in enumerate(data_dict.items()):
        ax1.plot(data['eef_0'], data['eef_1'], data['eef_2'], 
                color=colors[j], label=mode_name, linewidth=2)
        # Plot target trajectory if available
        # if target_dict[mode_name] is not None:
        #     ax1.plot(target_dict[mode_name]['target_eef_0'], target_dict[mode_name]['target_eef_1'], target_dict[mode_name]['target_eef_2'], 
        #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
    if predefined_eef_target_dict is not None:
        ax1.plot(predefined_eef_target_dict['eef_target_0'], predefined_eef_target_dict['eef_target_1'], predefined_eef_target_dict['eef_target_2'], 
                color='purple', linestyle='--', alpha=0.7, linewidth=0.5, label='Predefined EEF Target')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Left Hand 3D Trajectory Comparison')
    ax1.legend()
    
    # Right hand 3D trajectory
    ax2 = fig3.add_subplot(122, projection='3d')
    for j, (mode_name, data) in enumerate(data_dict.items()):
        ax2.plot(data['eef_7'], data['eef_8'], data['eef_9'], 
                color=colors[j], label=mode_name, linewidth=2)
        # Plot target trajectory if available
        # if target_dict[mode_name] is not None:
        #     ax2.plot(target_dict[mode_name]['target_eef_7'], target_dict[mode_name]['target_eef_8'], target_dict[mode_name]['target_eef_9'], 
        #             color=target_colors[j], linestyle='--', alpha=0.7, linewidth=1.5)
    if predefined_eef_target_dict is not None:
        ax2.plot(predefined_eef_target_dict['eef_target_7'], predefined_eef_target_dict['eef_target_8'], predefined_eef_target_dict['eef_target_9'], 
                color='purple', linestyle='--', alpha=0.7, linewidth=0.5, label='Predefined EEF Target')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_title('Right Hand 3D Trajectory Comparison')
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(csv_folder, 'eef_3d_trajectory_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"3D trajectory comparison plot saved: {os.path.join(csv_folder, 'eef_3d_trajectory_comparison.png')}")
    
    # Calculate and display tracking error statistics
    print("\n=== Tracking Performance Statistics ===")
    for mode_name, data in data_dict.items():
        print(f"{mode_name}:")
        
        if target_dict[mode_name] is not None:
            # Calculate RMSE against target trajectory
            rmse_x = np.sqrt(np.mean((data['state_0'] - target_dict[mode_name]['target_state_0'])**2))
            rmse_y = np.sqrt(np.mean((data['state_1'] - target_dict[mode_name]['target_state_1'])**2))
            rmse_z = np.sqrt(np.mean((data['state_2'] - target_dict[mode_name]['target_state_2'])**2))
            
            print(f"  Base X-axis RMSE vs Target: {rmse_x:.4f} m")
            print(f"  Base Y-axis RMSE vs Target: {rmse_y:.4f} m")
            print(f"  Base Z-axis RMSE vs Target: {rmse_z:.4f} m")
            print(f"  Overall Position RMSE vs Target: {np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2):.4f} m")
            
            # Calculate end-effector RMSE
            eef_rmse_x = np.sqrt(np.mean((data['eef_0'] - target_dict[mode_name]['target_eef_0'])**2))
            eef_rmse_y = np.sqrt(np.mean((data['eef_1'] - target_dict[mode_name]['target_eef_1'])**2))
            eef_rmse_z = np.sqrt(np.mean((data['eef_2'] - target_dict[mode_name]['target_eef_2'])**2))
            
            print(f"  Left Hand X-axis RMSE vs Target: {eef_rmse_x:.4f} m")
            print(f"  Left Hand Y-axis RMSE vs Target: {eef_rmse_y:.4f} m")
            print(f"  Left Hand Z-axis RMSE vs Target: {eef_rmse_z:.4f} m")
            print(f"  Left Hand Overall RMSE vs Target: {np.sqrt(eef_rmse_x**2 + eef_rmse_y**2 + eef_rmse_z**2):.4f} m")
        else:
            # Fallback to sinusoidal ideal trajectory
            time = data['time'].values
            ideal_x = 0.5 * np.sin(0.5 * time)
            ideal_y = 0.3 * np.cos(0.5 * time)
            ideal_z = 0.1 * np.sin(0.8 * time)
            
            rmse_x = np.sqrt(np.mean((data['state_0'] - ideal_x)**2))
            rmse_y = np.sqrt(np.mean((data['state_1'] - ideal_y)**2))
            rmse_z = np.sqrt(np.mean((data['state_2'] - ideal_z)**2))
            
            print(f"  Base X-axis RMSE: {rmse_x:.4f} m")
            print(f"  Base Y-axis RMSE: {rmse_y:.4f} m")
            print(f"  Base Z-axis RMSE: {rmse_z:.4f} m")
            print(f"  Overall Position RMSE: {np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2):.4f} m")
        
        print()
    
    plt.show()

if __name__ == "__main__":
    # Check if CSV folder path parameter is provided
    import sys
    if len(sys.argv) > 1:
        csv_folder = sys.argv[1]
    else:
        csv_folder = "/tmp"
    
    print(f"Reading CSV files from folder: {csv_folder}")
    plot_tracking_results(csv_folder) 