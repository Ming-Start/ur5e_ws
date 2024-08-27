import rospy
import sys
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from shapely.geometry import box, Point

def update_scene(scene):
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    # 定义障碍物的名称列表
    obstacle_names = ['cubic_obstacle', 'cubic_obstacle_1', 'cubic_obstacle_2', 'cubic_obstacle_3']
    obstacles = []  # 用于存储障碍物的位姿和尺寸

    for obstacle_name in obstacle_names:
        try:
            # 获取每个障碍物的状态（位置和姿态）
            obstacle_state = get_model_state(obstacle_name, 'world')
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get model state for {obstacle_name}: {e}")
            continue

        # 定义障碍物的大小 (根据你的URDF文件)
        box_size = (0.2, 0.2, 0.2)  # 假设立方体的尺寸为 0.2x0.2x0.2 米

        # 将障碍物的位姿和大小添加到列表中
        obstacles.append((obstacle_state.pose, box_size))

        # 创建一个 PoseStamped 对象来表示障碍物的位置，并更新到 MoveIt! 场景
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose = obstacle_state.pose

        # 将障碍物添加到 MoveIt! 场景中
        scene.add_box(obstacle_name, box_pose, size=box_size)

    # 添加地面作为障碍物
    ground_pose = PoseStamped()
    ground_pose.header.frame_id = "world"
    ground_pose.pose.position.x = 0
    ground_pose.pose.position.y = 0
    ground_pose.pose.position.z = 0  # 假设地面在 z=0 处

    ground_size = (2, 2, 0.01)  # 定义地面的大小和厚度
    scene.add_box("ground", ground_pose, size=ground_size)

    return obstacles  # 返回障碍物列表

def is_target_in_obstacle(target_position, obstacles):
    for obstacle_pose, obstacle_size in obstacles:
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
    
    waypoints = []
    waypoints.append(move_group.get_current_pose().pose)
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0)

    return fraction < 1.0

def move_to_safe_position(move_group):
    # 定义一个安全位置的关节角度值（以弧度为单位）
    safe_joint_position = [0, -1.57, 0, -1.57, 0, 0]  # 根据你的机器人实际的安全关节角度来设置

    # 设置关节目标
    move_group.set_joint_value_target(safe_joint_position)
    
    # 移动到目标关节位置
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def move_backwards(move_group, distance):
    # 简单向后移动机械臂来避开碰撞
    current_pose = move_group.get_current_pose().pose
    current_pose.position.z -= distance
    move_group.set_pose_target(current_pose)
    move_group.go(wait=True)
    move_group.stop()

def try_planning_with_different_planners(move_group, planners, target_position, scene, robot, max_attempts=3):
    move_group.set_planning_time(10)

    best_plan = None
    for attempt in range(max_attempts):
        for planner_id in planners:
            rospy.loginfo(f"Attempt {attempt + 1}/{max_attempts} - Trying planner: {planner_id}")
            move_group.set_planner_id(planner_id)
            
            # 设置目标为笛卡尔坐标
            target_pose = move_group.get_current_pose().pose
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            move_group.set_pose_target(target_pose)

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

            for _ in range(50):  # 动态检查碰撞
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

def move_to_position(x, y, z, move_to_safe=True):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    # 根据 move_to_safe 参数决定是否先移动到安全位置
    if move_to_safe:
        move_to_safe_position(move_group)

    target_position = [x, y, z]

    # 更新场景并获取所有障碍物的信息
    obstacles = update_scene(scene)

    # 检查目标点是否在任何一个障碍物内
    if is_target_in_obstacle(target_position, obstacles):
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
    # 第一次运行时，先移动到安全位置
    # move_to_position(0.5, 0.5, 0.2, move_to_safe=True)

    # 后续运行时，可以选择直接从当前位置移动到目标位置
    # move_to_position(-0.5, 0.5, 0.2, move_to_safe=False)
    move_to_position(0.4, 0.6, 0.5, move_to_safe=False)