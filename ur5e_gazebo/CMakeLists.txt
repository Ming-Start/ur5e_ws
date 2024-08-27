cmake_minimum_required(VERSION 3.0.2)
project(ur5e_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_commander
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装Python脚本（包括主脚本和模块）
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/move_viewpoints.py
  scripts/move_box.py
  scripts/avoid.py
  scripts/avoid_1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装Python模块
install(DIRECTORY obstacle_avoidance/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/obstacle_avoidance
  FILES_MATCHING PATTERN "*.py"
)
