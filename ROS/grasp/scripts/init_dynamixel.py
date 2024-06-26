#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))

import rospy
import numpy as np
from dynamixel_client import DynamixelClient
from grasp.srv import GetPosition, GetPositionResponse, SetPosition, SetPositionResponse, ResetMotor, ResetMotorResponse, SetCurrent, SetCurrentResponse, GetVelocity, GetVelocityResponse # type: ignore
from std_msgs.msg import Float64MultiArray  

def init():
    global dxl_client, redundant_dxl_client, motor_ids, redundant_motor_ids
    redundant_motor_ids = [12, 14, 22, 24]
    motor_ids = [11, 13, 21, 23]
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    baudrate = 57600
    dxl_client = None
    redundant_dxl_client = None

    for port in ports:
        try:
            dxl_client = DynamixelClient(motor_ids, port=port, baudrate=baudrate) 
            redundant_dxl_client = DynamixelClient(redundant_motor_ids, port=port, baudrate=baudrate)  
            dxl_client.connect()
            redundant_dxl_client.connect()
            rospy.loginfo(f'Dynamixel motors connected on {port}.')
            break
        except Exception as e:
            rospy.logwarn(f'Error connecting to Dynamixel motors on {port}: {str(e)}')

    if not dxl_client or not dxl_client.is_connected:
        rospy.logerr('Failed to connect to any Dynamixel motors.')
        rospy.signal_shutdown('Failed to connect to any Dynamixel motors.')
    return 
        
def fix():
    desired_positions = np.array([np.pi, np.pi, np.pi, np.pi])
    ADDR_SET_MODE = 11
    LEN_SET_MODE = 1
    redundant_dxl_client.sync_write(redundant_motor_ids, np.ones(len(redundant_motor_ids))*3, ADDR_SET_MODE, LEN_SET_MODE)
    redundant_dxl_client.set_torque_enabled(redundant_motor_ids, True)
    redundant_dxl_client.write_desired_pos(redundant_motor_ids, desired_positions)
    node = DynamixelServiceNode(dxl_client)
    node.spin()

class DynamixelServiceNode:

    def __init__(self, dxl_client):
        self.dxl_client = dxl_client

        # Initialize publisher
        self.pub = rospy.Publisher('motor_positions', Float64MultiArray, queue_size=10)

        rospy.Service('get_position', GetPosition, self.handle_get_position)
        rospy.Service('get_velocity', GetVelocity, self.handle_get_velocity)
        rospy.Service('set_position', SetPosition, self.handle_set_position)
        rospy.Service('reset_motor', ResetMotor, self.handle_reset_motor)
        rospy.Service('set_current', SetCurrent, self.handle_set_current)
        
        # Call and publish position periodically
        rospy.Timer(rospy.Duration(1.0/10), self.publish_position)

    def handle_get_position(self, req):
        pos = self.dxl_client.read_pos()
        # This sets the frame as per the theory used for this work.
        pos[0] -= np.pi/2
        pos[1] -= np.pi
        pos[2] -= np.pi/2
        pos[3] -= np.pi
        positions = [pos[0], pos[1], pos[2], pos[3]]
        return GetPositionResponse(positions)
    
    def handle_get_velocity(self, req):
        vel = self.dxl_client.read_vel()
        velocity = [vel[0], vel[1], vel[2], vel[3]]
        return GetVelocityResponse(velocity)
    
    def handle_set_position(self, req): 
        desired_positions = np.array(list(req.desired_positions))
        mode = 3
        ADDR_SET_MODE = 11
        LEN_SET_MODE = 1
        self.dxl_client.sync_write(motor_ids, np.ones(len(motor_ids))*mode, ADDR_SET_MODE, LEN_SET_MODE)
        self.dxl_client.set_torque_enabled(motor_ids, True)
        self.dxl_client.write_desired_pos(motor_ids, desired_positions)
        rospy.loginfo(f"Positions are set to desired value.")
        return SetPositionResponse(True)
    
    def handle_reset_motor(self, req): 
        self.dxl_client.set_torque_enabled(motor_ids, False)
        rospy.loginfo(f"Motors are disabled and ready for reset")
        return ResetMotorResponse(True)
    
    def handle_set_current(self, req): 
        desired_currents = np.array(list(req.desired_currents))
        mode = 0
        ADDR_SET_MODE = 11
        LEN_SET_MODE = 1
        self.dxl_client.sync_write(motor_ids, np.ones(len(motor_ids))*mode, ADDR_SET_MODE, LEN_SET_MODE)
        self.dxl_client.set_torque_enabled(motor_ids, True)
        self.dxl_client.write_desired_cur(motor_ids, desired_currents)
        rospy.loginfo(f"Currents are set to desired value.")
        return SetCurrentResponse(True)

    def publish_position(self, event):
        # Call the 'get_position' service and publish the result
        try:
            rospy.wait_for_service('get_position')
            get_position = rospy.ServiceProxy('get_position', GetPosition)
            response = get_position()
            msg = Float64MultiArray(data=response.position)
            self.pub.publish(msg)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException as e:
            rospy.logwarn(f"Service not available: {e}")

    def spin(self):
        rospy.spin()

    def __del__(self):
        if self.dxl_client:
            self.dxl_client.disconnect()
            rospy.loginfo('Dynamixel motors disconnected.')

def main():
    rospy.init_node('dynamixel_service_node')
    init()
    fix()

if __name__ == '__main__':
    main()
