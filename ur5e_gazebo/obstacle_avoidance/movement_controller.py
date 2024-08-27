import rospy

def move_backwards(move_group, distance=0.1):
    current_pose = move_group.get_current_pose().pose
    current_pose.position.z += distance

    move_group.set_pose_target(current_pose)
    success, plan, _, _ = move_group.plan()

    if success:
        move_group.execute(plan, wait=True)
    else:
        rospy.logwarn("Failed to plan backward movement.")

def move_to_safe_position(move_group):
    safe_position = [0.0, 0.15, 1.0]
    move_group.set_position_target(safe_position)
    
    success, plan, _, _ = move_group.plan()
    
    if success:
        rospy.loginfo("Moving to safe position...")
        move_group.execute(plan, wait=True)
        rospy.loginfo("Moved to safe position.")
    else:
        rospy.logwarn("Failed to move to safe position.")