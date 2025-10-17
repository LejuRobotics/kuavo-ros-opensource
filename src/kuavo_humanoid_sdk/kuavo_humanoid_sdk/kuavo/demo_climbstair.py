#!/usr/bin/env python3
# coding: utf-8

import argparse
import sys
import rospy
from robot_climbstair import KuavoRobotClimbStair


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Stair climbing demo using KuavoRobotClimbStair"
    )
    parser.add_argument(
        "--plot", action="store_true", help="Enable trajectory plotting"
    )
    parser.add_argument(
        "--initH", type=float, default=0.0, help="Stand height offset (default: 0.0)"
    )
    return parser.parse_args()


def wait_for_user_confirmation(message: str) -> bool:
    """Wait for user confirmation before proceeding."""
    try:
        response = input(f"{message} (y/n): ").lower().strip()
        return response in ["y", "yes"]
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        return False


def main():
    """Main demo function replicating original stairClimbPlanner.py sequence."""
    try:
        # Parse command line arguments
        args = parse_args()
        plot_enabled = args.plot
        stand_height = args.initH

        rospy.loginfo(
            f"[Demo] Starting stair climbing demo with plot={plot_enabled}, stand_height={stand_height}"
        )

        # Initialize robot stair climbing system
        climb_stair = KuavoRobotClimbStair()

        # Set stair parameters matching the original script
        success = climb_stair.set_stair_parameters(
            step_height=0.13,  # 台阶高度
            step_length=0.28,  # 台阶长度
            foot_width=0.10,  # 宽
            stand_height=stand_height,
            dt=0.6,  # 步态周期
            ss_time=0.5,  # 支撑相时间
        )

        if not success:
            rospy.logerr("[Demo] Failed to set stair parameters")
            return False

        # Display current parameters
        params = climb_stair.get_parameters()
        rospy.loginfo(
            f"[Demo] Current parameters: step_height={params['step_height']:.3f}m, "
            f"step_length={params['step_length']:.3f}m, dt={params['dt']:.3f}s"
        )

        # Ask user confirmation before starting
        if not wait_for_user_confirmation("Start stair climbing demo sequence?"):
            rospy.loginfo("[Demo] Demo cancelled by user")
            return False

        # Use the new API: plan all trajectories, then publish them
        rospy.loginfo("[Demo] Planning complete demo sequence...")

        # Disable pitch limit for stair climbing
        from robot_climbstair import set_pitch_limit

        set_pitch_limit(False)

        # Clear any existing trajectory
        climb_stair.clear_trajectory()

        # Phase 1: Plan up stairs
        success = climb_stair.climb_up_stairs(5)
        if not success:
            rospy.logerr("[Demo] Failed to plan up stairs")
            set_pitch_limit(True)
            return False
        rospy.loginfo("[Demo] Up stairs plan done.")

        # Phase 2: Plan move forward
        success = climb_stair.move_to_position(0.35, 0, 0)
        if not success:
            rospy.logerr("[Demo] Failed to plan move forward")
            set_pitch_limit(True)
            return False
        rospy.loginfo("[Demo] Move forward plan done.")

        # Phase 3: Plan down stairs (TEMPORARILY DISABLED)
        rospy.loginfo(
            "[Demo] Skipping down stairs phase (functionality under development)"
        )
        # success = climb_stair.climb_down_stairs(5)
        # if not success:
        #     rospy.logerr("[Demo] Failed to plan down stairs")
        #     set_pitch_limit(True)
        #     return False
        rospy.loginfo("[Demo] Down stairs phase skipped.")

        # Execute the complete accumulated trajectory
        rospy.loginfo("[Demo] Executing complete trajectory sequence...")
        success = climb_stair.execute_trajectory()

        # Re-enable pitch limit
        set_pitch_limit(True)

        if not success:
            rospy.logerr("[Demo] Failed to execute demo sequence")
            return False

        rospy.loginfo("[Demo] ✓ Complete demo sequence executed successfully!")

        # Print final statistics
        total_steps = climb_stair.get_step_count()
        rospy.loginfo(
            f"[Demo] Demo completed successfully! Total steps taken: {total_steps}"
        )
        print(f"\n=== DEMO COMPLETED ===")
        print(f"Total steps taken: {total_steps}")
        print("All phases completed successfully!")

        return True

    except KeyboardInterrupt:
        rospy.logwarn("[Demo] Demo interrupted by user (Ctrl+C)")
        return False
    except Exception as e:
        rospy.logerr(f"[Demo] Demo failed with exception: {e}")
        return False


def run_detailed_demo():
    """Run demo with detailed trajectory logging (similar to original script)."""
    try:
        args = parse_args()
        stand_height = args.initH

        rospy.loginfo("[Demo] Starting detailed stair climbing demo")

        # Initialize robot
        climb_stair = KuavoRobotClimbStair()

        # Set parameters
        climb_stair.set_stair_parameters(
            step_height=0.13,
            step_length=0.28,
            foot_width=0.10,
            stand_height=stand_height,
        )

        print("\n=== DETAILED STAIR CLIMBING DEMO ===")
        print("This demo replicates the exact sequence from stairClimbPlanner.py:")
        print("1. Up stairs (5 steps)")
        print("2. Move forward (0.35m)")
        print("3. Down stairs (5 steps)")
        print()

        # Disable pitch limit (matching original behavior)
        climb_stair.set_pitch_limit(False)

        print("=== EXECUTING COMPLETE TRAJECTORY SEQUENCE ===")

        # Use the new API: plan all trajectories, then publish them
        climb_stair.clear_trajectory()

        # Phase 1: Plan up stairs
        success = climb_stair.climb_up_stairs(5)
        if not success:
            print("✗ Failed to plan up stairs")
            return False
        print("✓ Up stairs plan done.")

        # Phase 2: Plan move forward
        success = climb_stair.move_to_position(0.35, 0, 0)
        if not success:
            print("✗ Failed to plan move forward")
            return False
        print("✓ Move forward plan done.")

        # Phase 3: Plan down stairs (TEMPORARILY DISABLED)
        print("⚠ Skipping down stairs phase (functionality under development)")
        # success = climb_stair.climb_down_stairs(5)
        # if not success:
        #     print("✗ Failed to plan down stairs")
        #     return False
        print("✓ Down stairs phase skipped.")

        # Execute the complete accumulated trajectory
        print("Executing complete trajectory sequence...")
        success = climb_stair.execute_trajectory()
        if success:
            print("✓ Complete trajectory sequence executed successfully")
            print(f"Total step count: {climb_stair.get_step_count()}")
        else:
            print("✗ Complete trajectory sequence failed")
            return False

        # Re-enable pitch limit (safety)
        climb_stair.set_pitch_limit(True)

        total_steps = climb_stair.get_step_count()
        print(f"\n=== DEMO COMPLETED ===")
        print(f"Total steps taken: {total_steps}")
        print("All phases completed successfully!")
        print("\nTrajectory published to: /humanoid_mpc_foot_pose_target_trajectories")

        return True

    except Exception as e:
        # Ensure pitch limit is restored on error
        try:
            climb_stair.set_pitch_limit(True)
        except:
            pass
        rospy.logerr(f"[Demo] Detailed demo failed: {e}")
        return False


if __name__ == "__main__":
    try:
        rospy.init_node("stair_climbing_demo", anonymous=True)
        success = main()

        if success:
            rospy.loginfo("[Demo] Demo completed successfully!")
            sys.exit(0)
        else:
            rospy.logerr("[Demo] Demo failed!")
            sys.exit(1)

    except KeyboardInterrupt:
        rospy.logwarn("[Demo] Demo interrupted by user")
        sys.exit(130)
    except Exception as e:
        rospy.logerr(f"[Demo] Unexpected error: {e}")
        sys.exit(1)
