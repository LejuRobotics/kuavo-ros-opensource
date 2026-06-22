#!/usr/bin/env python3
# coding: utf-8

import argparse
import sys
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.kuavo.robot_climbstair import KuavoRobotClimbStair, set_pitch_limit


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

        SDKLogger.info(
            f"[Demo] Starting stair climbing demo with plot={plot_enabled}, stand_height={stand_height}"
        )

        # Initialize robot stair climbing system
        climb_stair = KuavoRobotClimbStair()

        # Set stair parameters matching the original script
        success = climb_stair.set_stair_parameters(
            step_height=0.13,
            step_length=0.28,
            foot_width=0.10,
            stand_height=stand_height,
            dt=0.6,
            ss_time=0.5,
        )

        if not success:
            SDKLogger.error("[Demo] Failed to set stair parameters")
            return False

        # Display current parameters
        params = climb_stair.get_parameters()
        SDKLogger.info(
            f"[Demo] Current parameters: step_height={params['step_height']:.3f}m, "
            f"step_length={params['step_length']:.3f}m, dt={params['dt']:.3f}s"
        )

        # Ask user confirmation before starting
        if not wait_for_user_confirmation("Start stair climbing demo sequence?"):
            SDKLogger.info("[Demo] Demo cancelled by user")
            return False

        # Use the new API: plan all trajectories, then publish them
        SDKLogger.info("[Demo] Planning complete demo sequence...")

        # Disable pitch limit for stair climbing
        set_pitch_limit(False)

        # Clear any existing trajectory
        climb_stair.clear_trajectory()

        # Phase 1: Plan up stairs
        success = climb_stair.climb_up_stairs(5)
        if not success:
            SDKLogger.error("[Demo] Failed to plan up stairs")
            set_pitch_limit(True)
            return False
        SDKLogger.info("[Demo] Up stairs plan done.")

        # Phase 2: Plan move forward
        success = climb_stair.move_to_position(0.35, 0, 0)
        if not success:
            SDKLogger.error("[Demo] Failed to plan move forward")
            set_pitch_limit(True)
            return False
        SDKLogger.info("[Demo] Move forward plan done.")

        # Phase 3: Plan down stairs (TEMPORARILY DISABLED)
        SDKLogger.info(
            "[Demo] Skipping down stairs phase (functionality under development)"
        )
        SDKLogger.info("[Demo] Down stairs phase skipped.")

        # Execute the complete accumulated trajectory
        SDKLogger.info("[Demo] Executing complete trajectory sequence...")
        success = climb_stair.execute_trajectory()

        # Re-enable pitch limit
        set_pitch_limit(True)

        if not success:
            SDKLogger.error("[Demo] Failed to execute demo sequence")
            return False

        SDKLogger.info("[Demo] Complete demo sequence executed successfully!")

        # Print final statistics
        total_steps = climb_stair.get_step_count()
        SDKLogger.info(
            f"[Demo] Demo completed successfully! Total steps taken: {total_steps}"
        )
        print(f"\n=== DEMO COMPLETED ===")
        print(f"Total steps taken: {total_steps}")
        print("All phases completed successfully!")

        return True

    except KeyboardInterrupt:
        SDKLogger.warn("[Demo] Demo interrupted by user (Ctrl+C)")
        return False
    except Exception as e:
        SDKLogger.error(f"[Demo] Demo failed with exception: {e}")
        return False


def run_detailed_demo():
    """Run demo with detailed trajectory logging (similar to original script)."""
    try:
        args = parse_args()
        stand_height = args.initH

        SDKLogger.info("[Demo] Starting detailed stair climbing demo")

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
            print("Failed to plan up stairs")
            return False
        print("Up stairs plan done.")

        # Phase 2: Plan move forward
        success = climb_stair.move_to_position(0.35, 0, 0)
        if not success:
            print("Failed to plan move forward")
            return False
        print("Move forward plan done.")

        # Phase 3: Plan down stairs (TEMPORARILY DISABLED)
        print("Skipping down stairs phase (functionality under development)")
        print("Down stairs phase skipped.")

        # Execute the complete accumulated trajectory
        print("Executing complete trajectory sequence...")
        success = climb_stair.execute_trajectory()
        if success:
            print("Complete trajectory sequence executed successfully")
            print(f"Total step count: {climb_stair.get_step_count()}")
        else:
            print("Complete trajectory sequence failed")
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
        SDKLogger.error(f"[Demo] Detailed demo failed: {e}")
        return False


if __name__ == "__main__":
    try:
        success = main()

        if success:
            SDKLogger.info("[Demo] Demo completed successfully!")
            sys.exit(0)
        else:
            SDKLogger.error("[Demo] Demo failed!")
            sys.exit(1)

    except KeyboardInterrupt:
        SDKLogger.warn("[Demo] Demo interrupted by user")
        sys.exit(130)
    except Exception as e:
        SDKLogger.error(f"[Demo] Unexpected error: {e}")
        sys.exit(1)
