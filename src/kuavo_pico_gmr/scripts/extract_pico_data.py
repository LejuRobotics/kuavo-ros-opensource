"""
Extract and organize data from pico_data.bag ROS bag file.
This script extracts all topics and saves them in an organized format.
"""

import os
import json
import pandas as pd
from bagpy import bagreader
from pathlib import Path
import numpy as np
import rosbag


PICO_BONE_NAMES = [
    'pelvis',
    'left_hip',
    'right_hip',
    'spine1',
    'left_knee',
    'right_knee',
    'spine2',
    'left_ankle',
    'right_ankle',
    'spine3',
    'left_foot',
    'right_foot',
    'neck',
    'left_collar',
    'right_collar',
    'head',
    'left_shoulder',
    'right_shoulder',
    'left_elbow',
    'right_elbow',
    'left_wrist',
    'right_wrist',
    'left_hand',
    'right_hand',
]

def extract_bone_poses_with_names(bag_path, output_path, pose_topic='/pico/world_bone_poses'):
    """ Extract bone poses with bone names for /leju_pico_bone_poses topic. """
    print(f"  Extracting bone poses with names...")
    rows = []
    bag = rosbag.Bag(bag_path)
    
    for topic, msg, t in bag.read_messages(topics=[pose_topic]):
        timestamp = t.to_sec()
        timestamp_ms = msg.timestamp_ms
        is_high_confidence = msg.is_high_confidence
        is_hand_tracking = msg.is_hand_tracking
        
        for bone_idx, pose in enumerate(msg.poses):
            bone_name = PICO_BONE_NAMES[bone_idx] if bone_idx < len(PICO_BONE_NAMES) else f'Bone_{bone_idx}'
            row = {
                'Time': timestamp,
                'timestamp_ms': timestamp_ms,
                'is_high_confidence': is_high_confidence,
                'is_hand_tracking': is_hand_tracking,
                'bone_index': bone_idx,
                'bone_name': bone_name,
                'position_x': pose.position.x,
                'position_y': pose.position.y,
                'position_z': pose.position.z,
                'orientation_x': pose.orientation.x,
                'orientation_y': pose.orientation.y,
                'orientation_z': pose.orientation.z,
                'orientation_w': pose.orientation.w,
            }
            rows.append(row)
    
    bag.close()
    df = pd.DataFrame(rows)
    df.to_csv(output_path, index=False)
    print(f"  ✓ Saved {len(df)} bone pose records with names to: {output_path}")
    return df

def extract_bag_data(
    bag_path='./pico_data.bag', 
    output_dir='./pico_data',
    output_file_name='pico_data.csv',
    pose_topic='/pico/world_bone_poses',
):
    """
    Extract all data from ROS bag file and organize it.
    
    Args:
        bag_path: Path to the bag file
        output_dir: Directory to save extracted data
    """
    print(f"Reading bag file: {bag_path}")
    b = bagreader(bag_path)
    
    # Create output directory
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # Print topic information
    print("\n" + "="*80)
    print("TOPIC INFORMATION")
    print("="*80)
    print(b.topic_table)
    
    # Extract data for each topic
    print("\n" + "="*80)
    print("EXTRACTING DATA FROM TOPICS")
    print("="*80)
    
    topic_summary = {}
    
    for topic in b.topic_table['Topics']:
        print(f"\nProcessing topic: {topic}")
        try:
            #### Get messages for this topic (returns file path)
            # csv_path = b.message_by_topic(topic)
            
            # Special handling for bone poses topic - extract with bone names
            if topic == pose_topic:
                # Also extract with bone names
                bone_names_path = os.path.join(output_dir, output_file_name)
                try:
                    extract_bone_poses_with_names(bag_path, bone_names_path, pose_topic)
                except Exception as e:
                    print(f"  ⚠ Could not extract with bone names: {str(e)}")
                
        except Exception as e:
            print(f"  ✗ Error processing topic {topic}: {str(e)}")
            import traceback
            traceback.print_exc()
            if topic not in topic_summary:
                topic_summary[topic] = {
                    'status': 'error',
                    'error': str(e)
                }
    
    print("\n" + "="*80)
    print("EXTRACTION COMPLETE")
    print("="*80)
    print(f"All data extracted to: {output_dir}")
    
    return topic_summary


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Extract and organize data from ROS bag file')
    parser.add_argument('--bag_file', type=str, default='./rosbag_data/eval_data.bag',
                        help='Path to the bag file (default: ./rosbag_data/pico_data_ori.bag)')
    parser.add_argument('--output_dir', type=str, default='./pico_data',
                        help='Output directory for extracted data (default: ./pico_data)')
    parser.add_argument('--output_file_name', type=str, default='eval_data.csv',
                        help='Output file name for extracted data (default: pico_data_ori.csv)')
    parser.add_argument('--pose_topic', type=str, default='/pico/world_bone_poses',
                        help='Topic name for extracted data (default: /pico/world_bone_poses)')
    args = parser.parse_args()
    
    extract_bag_data(args.bag_file, args.output_dir, args.output_file_name, args.pose_topic)
