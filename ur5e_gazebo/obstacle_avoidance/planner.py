import rospy
from movement_controller import move_backwards, move_to_safe_position
from collision_checker import is_in_collision
from scene_manager import update_scene

def try_planning_with_different_planners(move_group, planners, target_position, scene, robot, max_attempts=3):
    move_group.set_planning_time(10)

    best_plan = None
    for attempt in range(max_attempts):
        for planner_id in planners:
            rospy.loginfo(f"Attempt {attempt + 1}/{max_attempts} - Trying planner: {planner_id}")
            move_group.set_planner_id(planner_id)
            
            move_group.set_position_target(target_position)
            success, plan, _, _ = move_group.plan()

            if success:
                rospy.loginfo("Checking for collisions before executing plan...")
                if is_in_collision(scene, robot, move_group):
                    rospy.logwarn("Pre-execution collision detected! Aborting and trying next planner.")
                    continue
                
                if best_plan is None or len(plan.joint_trajectory.points) < len(best_plan.joint_trajectory.points):
                    best_plan = plan

        if best_plan:
            move_group.execute(best_plan, wait=False)

            for _ in range(50):
                update_scene(scene)
                if is_in_collision(scene, robot, move_group):
                    rospy.logwarn("Collision detected during execution! Stopping the robot...")
                    
                    move_group.stop()
                    move_group.clear_pose_targets()

                    move_backwards(move_group, distance=0.1)
                    
                    return try_planning_with_different_planners(move_group, planners, target_position, scene, robot, max_attempts)
                
                rospy.sleep(0.2)

            rospy.loginfo(f"Successfully moved to position using {planner_id}.")
            return True
        else:
            rospy.logwarn(f"All planners failed on attempt {attempt + 1}. Moving to safe position and retrying...")
            move_to_safe_position(move_group)

    rospy.logerr("All planning attempts failed. Unable to reach the target position.")
    return False