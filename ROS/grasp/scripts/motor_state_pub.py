#!/usr/bin/env python3

import rospy
from grasp.srv import GetPosition, GetVelocity # type: ignore
from std_msgs.msg import Float64MultiArray

def get_motor_position():
    rospy.wait_for_service('get_position')
    try:
        get_position = rospy.ServiceProxy('get_position', GetPosition)
        response = get_position()
        return response.position
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return []

def get_motor_velocity():
    rospy.wait_for_service('get_velocity')
    try:
        get_velocity = rospy.ServiceProxy('get_velocity', GetVelocity)
        response = get_velocity()
        return response.velocity
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return []

def main():
    rospy.init_node('read_motor_data')
    rospy.loginfo("Client node initialized.")

    # Publishers for positions and velocities
    position_pub = rospy.Publisher('motor_positions', Float64MultiArray, queue_size=10)
    velocity_pub = rospy.Publisher('motor_velocities', Float64MultiArray, queue_size=10)

    rospy.loginfo("Publishing motor positions and velocities...")
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Get motor positions and velocities
        positions = get_motor_position()
        velocities = get_motor_velocity()

        # Publish positions
        if positions:
            position_msg = Float64MultiArray(data=positions)
            position_pub.publish(position_msg)

        # Publish velocities
        if velocities:
            velocity_msg = Float64MultiArray(data=velocities)
            velocity_pub.publish(velocity_msg)

        rate.sleep()

if __name__ == '__main__':
    main()
