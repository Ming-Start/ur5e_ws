#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import threading

def move_box():
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    state_msg = ModelState()
    state_msg.model_name = 'cubic_obstacle'
    state_msg.pose.position.x = 0.5
    state_msg.pose.position.z = 0.45
    state_msg.pose.orientation.x = 0.0
    state_msg.pose.orientation.y = 0.0
    state_msg.pose.orientation.z = 0.0
    state_msg.pose.orientation.w = 1.0

    rate = rospy.Rate(60)  # 60 Hz
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # Compute a new Y position using a sine wave for smooth back-and-forth motion
        new_y_position = 0.1 * math.sin(0.5 * elapsed_time)

        # Only update if the position has changed significantly
        if abs(state_msg.pose.position.y - new_y_position) > 0.001:
            state_msg.pose.position.y = new_y_position
            try:
                set_model_state(state_msg)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('move_box_node', anonymous=True)
    try:
        thread = threading.Thread(target=move_box)
        thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass