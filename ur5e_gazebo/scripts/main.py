import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander

from obstacle_avoidance.scene_manager import update_scene
from obstacle_avoidance.collision_checker import is_target_in_obstacle
from obstacle_avoidance.planner import try_planning_with_different_planners


def move_to_position(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    target_position = [x, y, z]

    obstacle_pose, obstacle_size = update_scene(scene)

    if is_target_in_obstacle(target_position, obstacle_pose, obstacle_size):
        rospy.logerr(f"Target position ({x}, {y}, {z}) is inside an obstacle. Please choose a different target.")
        return

    planners = [
        "RRTConnectkConfigDefault",
        "RRTstar",
        "PRMkConfigDefault"
    ]

    success = try_planning_with_different_planners(move_group, planners, target_position, scene, robot)

    if success:
        rospy.loginfo(f"Successfully moved to position ({x}, {y}, {z}) while avoiding obstacles.")
    else:
        rospy.logerr(f"Failed to move to position ({x}, {y}, {z}) while avoiding obstacles.")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    move_to_position(0.5, 0.5, 0.2)
    # move_to_position(0.6, 0.4, 0.8)