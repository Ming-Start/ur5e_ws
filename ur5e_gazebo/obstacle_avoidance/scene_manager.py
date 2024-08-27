import rospy
from moveit_commander import PlanningSceneInterface
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped

def update_scene(scene):
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    obstacle_name = 'cubic_obstacle'
    try:
        obstacle_state = get_model_state(obstacle_name, 'world')
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to get model state for {obstacle_name}: {e}")
        return

    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose = obstacle_state.pose

    safety_margin = 0.1
    box_size = (0.2 + safety_margin, 0.2 + safety_margin, 0.2 + safety_margin)
    scene.add_box(obstacle_name, box_pose, size=box_size)

    ground_pose = PoseStamped()
    ground_pose.header.frame_id = "world"
    ground_pose.pose.position.x = 0
    ground_pose.pose.position.y = 0
    ground_pose.pose.position.z = 0

    ground_size = (2, 2, 0.01)
    scene.add_box("ground", ground_pose, size=ground_size)

    return box_pose.pose, box_size