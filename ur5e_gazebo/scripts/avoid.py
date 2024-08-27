import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import random
from moveit_commander import PlanningSceneInterface, RobotCommander
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from shapely.geometry import Point, box  # 需要安装 shapely 库

# 设置随机种子以确保路径规划一致
RANDOM_SEED = 42
random.seed(RANDOM_SEED)

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

    # 增加障碍物的安全边界
    safety_margin = 0.1  # 例如，增加10cm的安全边界
    box_size = (0.2 + safety_margin, 0.2 + safety_margin, 0.2 + safety_margin)
    scene.add_box(obstacle_name, box_pose, size=box_size)

    # 添加地面为障碍物
    ground_pose = PoseStamped()
    ground_pose.header.frame_id = "world"
    ground_pose.pose.position.x = 0
    ground_pose.pose.position.y = 0
    ground_pose.pose.position.z = 0  # 假设地面在z=0处

    ground_size = (2, 2, 0.01)  # 定义地面的大小和厚度
    scene.add_box("ground", ground_pose, size=ground_size)

    return box_pose.pose, box_size  # 返回障碍物的位置信息

def is_target_in_obstacle(target_position, obstacle_pose, obstacle_size):
    # 使用 shapely 库检查目标点是否在障碍物内
    obs_min_x = obstacle_pose.position.x - obstacle_size[0] / 2
    obs_max_x = obstacle_pose.position.x + obstacle_size[0] / 2
    obs_min_y = obstacle_pose.position.y - obstacle_size[1] / 2
    obs_max_y = obstacle_pose.position.y + obstacle_size[1] / 2
    obs_min_z = obstacle_pose.position.z - obstacle_size[2] / 2
    obs_max_z = obstacle_pose.position.z + obstacle_size[2] / 2

    obstacle_box = box(obs_min_x, obs_min_y, obs_max_x, obs_max_y)
    target_point = Point(target_position[0], target_position[1])

    # 检查 x, y 是否在障碍物的范围内
    if obstacle_box.contains(target_point):
        # 检查 z 轴是否在障碍物的范围内
        if obs_min_z <= target_position[2] <= obs_max_z:
            return True
    return False

def is_in_collision(scene, robot, move_group):
    current_state = robot.get_current_state()
    move_group.set_start_state(current_state)
    
    # 通过规划路径并检查 fraction 来判断是否存在碰撞
    waypoints = []
    waypoints.append(move_group.get_current_pose().pose)
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,   # 途径点
        0.01,        # 末端执行器步长
        0.0)         # 跳跃阈值

    return fraction < 1.0  # 如果规划失败，可能存在碰撞

def move_backwards(move_group, distance=0.1):
    # 获取当前位姿
    current_pose = move_group.get_current_pose().pose
    
    # 修改位姿使得机械臂向后移动一定距离
    current_pose.position.z += distance

    # 规划反向移动路径
    move_group.set_pose_target(current_pose)
    success, plan, _, _ = move_group.plan()  # 解包返回值，确保 plan 是路径结果

    if success:
        move_group.execute(plan, wait=True)
    else:
        rospy.logwarn("Failed to plan backward movement.")

def move_to_safe_position(move_group):
    # 定义一个安全位置，假设为机械臂的初始位置或一个已知的安全位置
    safe_position = [0.0, 0.15, 1.0]
    move_group.set_position_target(safe_position)
    
    success, plan, _, _ = move_group.plan()
    
    if success:
        rospy.loginfo("Moving to safe position...")
        move_group.execute(plan, wait=True)
        rospy.loginfo("Moved to safe position.")
    else:
        rospy.logwarn("Failed to move to safe position.")

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
                
                # 选择路径最短的方案
                if best_plan is None or len(plan.joint_trajectory.points) < len(best_plan.joint_trajectory.points):
                    best_plan = plan

        # 执行最优的路径
        if best_plan:
            move_group.execute(best_plan, wait=False)

            for _ in range(50):
                update_scene(scene)
                if is_in_collision(scene, robot, move_group):
                    rospy.logwarn("Collision detected during execution! Stopping the robot...")
                    
                    # 停止机械臂
                    move_group.stop()
                    move_group.clear_pose_targets()
                    
                    # 向后移动一段安全距离
                    move_backwards(move_group, distance=0.1)
                    
                    # 重新规划路径
                    return try_planning_with_different_planners(move_group, planners, target_position, scene, robot, max_attempts)
                
                rospy.sleep(0.2)

            rospy.loginfo(f"Successfully moved to position using {planner_id}.")
            return True
        else:
            # 如果所有规划器都失败，移动到安全位置并重新尝试
            rospy.logwarn(f"All planners failed on attempt {attempt + 1}. Moving to safe position and retrying...")
            move_to_safe_position(move_group)

    rospy.logerr("All planning attempts failed. Unable to reach the target position.")
    return False

def move_to_position(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    target_position = [x, y, z]

    # 更新场景并获取障碍物信息
    obstacle_pose, obstacle_size = update_scene(scene)

    # 检查目标点是否在障碍物内
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