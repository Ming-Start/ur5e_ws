#!/usr/bin python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def move_to_position(x, y, z):
    # Initialize MoveIt! and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e', anonymous=True)

    # Initialize MoveGroupCommander
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # Define the target pose
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    # Set the target pose
    move_group.set_pose_target(pose_target)

    # Plan and execute the motion
    if move_group.go(wait=True):
        rospy.loginfo(f"Successfully moved to position ({x}, {y}, {z}).")
    else:
        rospy.logerr(f"Failed to move to position ({x}, {y}, {z}).")

    # Stop any residual movement and clear targets
    move_group.stop()
    move_group.clear_pose_targets()

    # Shutdown MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    # Example usage, replace with your desired coordinates
    move_to_position(0.6, 0.0, 0.6)
