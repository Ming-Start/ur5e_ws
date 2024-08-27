from shapely.geometry import Point, box

def is_target_in_obstacle(target_position, obstacle_pose, obstacle_size):
    obs_min_x = obstacle_pose.position.x - obstacle_size[0] / 2
    obs_max_x = obstacle_pose.position.x + obstacle_size[0] / 2
    obs_min_y = obstacle_pose.position.y - obstacle_size[1] / 2
    obs_max_y = obstacle_pose.position.y + obstacle_size[1] / 2
    obs_min_z = obstacle_pose.position.z - obstacle_size[2] / 2
    obs_max_z = obstacle_pose.position.z + obstacle_size[2] / 2

    obstacle_box = box(obs_min_x, obs_min_y, obs_max_x, obs_max_y)
    target_point = Point(target_position[0], target_position[1])

    if obstacle_box.contains(target_point):
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