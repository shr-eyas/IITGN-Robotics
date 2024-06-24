#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from grasp.srv import SetPosition, SetPositionRequest, ResetMotor, ResetMotorRequest, SetCurrent, SetCurrentRequest

'''
FUNCTIONS TO HANDLE MOTORS
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
def send_pose(positions):
    rospy.wait_for_service('set_position')
    try:
        set_position = rospy.ServiceProxy('set_position', SetPosition)
        req = SetPositionRequest()
        positions_ = np.array([np.pi/2, np.pi, np.pi/2, np.pi]) + np.array(positions)
        req.desired_positions = positions_
        resp = set_position(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def disable_motors():
    rospy.wait_for_service('reset_motor')
    try:
        reset_motor = rospy.ServiceProxy('reset_motor', ResetMotor)
        req = ResetMotorRequest()
        resp = reset_motor(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False
    
def send_current(currents):
    rospy.wait_for_service('set_current')
    try:
        set_current = rospy.ServiceProxy('set_current', SetCurrent)
        req = SetCurrentRequest()
        req.desired_currents = currents
        resp = set_current(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

'''
INITIALIZATIONS
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
dt = 1/100.0  # Time step
T = 1 # Total time for each trajectory
t = np.arange(0, T, dt)
n_samples = int(T/dt)  # Number of samples in the trajectory
freq = 1/dt  # Frequency of control loop
trials = 100
Kt1 = 0.35
Kt2 = 0.51

# The side length of the cube is set and used in Grasp Matrix and 
# Impedance Parameters
xDes = 0
yDes = 0.13
thetaDes = 0

KpX = 5000
KpY = 0
KpTheta = 50
Kp = np.array([[KpX, 0, 0], 
               [0, KpY, 0], 
               [0, 0, KpTheta]]) 

'''
DATA HANDLING
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
latest_data = {
    'object_position': None,
    'grasp_matrix': None,
    'hand_jacobian': None,
}
    
def aruco_callback(data):
    position = data.data 
    update_dictionary('object_position', position)

def grasp_callback(data):
    grasp = data.data
    update_dictionary('grasp_matrix', grasp)

def hand_jacobian_callback(data):
    hand_jac = data.data
    update_dictionary('hand_jacobian', hand_jac)

def update_dictionary(key, value):
    latest_data[key] = value

'''
MAIN LOOP 
________________________________________________________________________________________________________________________________________________
'''
def ilc():

    objectPosition = latest_data['object_position']
    handJacobianFlattened = latest_data['hand_jacobian']
    handJacobian = np.array(handJacobianFlattened).reshape((4, 4))
    graspMatrixFlattened = latest_data['grasp_matrix']
    graspMatrix = np.array(graspMatrixFlattened).reshape((3, 4))
   
# def trial_loop():

#     for i in range(trials):

#         home()
       
#         for j in range(0, n_samples-1):

#             Jh = utils.hand_jacobian(phi1, phi2, [0.103, 0.093], [0.103, 0.093], position[2], 0, 0)
#             handJacobian = np.array(Jh, dtype=float)

#             graspMatrix = self.grasp_matrix

#             f_null = np.dot((np.identity(4) - np.dot(np.linalg.pinv(graspMatrix), graspMatrix)), np.array([[200],[200],[200],[200]])) 
#             f_impedence = np.dot(np.linalg.pinv(graspMatrix), np.dot(Kp,(np.array([[xDes - position[0]], [yDes - position[1]], [thetaDes - position[2]]])))) 

#             fingerF = f_null + f_impedence
#             handJacobianT = np.transpose(handJacobian)
#             tau = np.dot(handJacobianT, fingerF)  # Fixed indexing issue
#             goalCurrent = tau 
#             goalCurrent[0] = goalCurrent[0]/Kt1
#             goalCurrent[1] = goalCurrent[1]/Kt2
#             goalCurrent[2] = goalCurrent[2]/Kt1
#             goalCurrent[3] = goalCurrent[3]/Kt2

#             send_pose([pose], dt)
           
#             rospy.sleep(dt)
                
#         rospy.sleep(3)
#         rospy.loginfo("Trial loop %d completed.", n + 1)
#         rospy.sleep(2)  # Rest for 2 seconds after each trial loop
    
'''
MAIN FUNCTION
________________________________________________________________________________________________________________________________________________
'''
# def main():
#     rospy.init_node('ilc_controller', anonymous=True)

#     rospy.Subscriber('aruco_data', Float64MultiArray, aruco_callback)
#     rospy.Subscriber('grasp_matrix', Float64MultiArray, grasp_callback)
#     rospy.Subscriber('hand_jacobian', Float64MultiArray, hand_jacobian_callback)

#     rospy.loginfo("Main node is up.")
#     rospy.loginfo("Waiting for data.")

#     while latest_data['grasp_matrix'] is None and latest_data['hand_jacobian'] is None:
#         rospy.sleep(0.1)

#     rospy.loginfo("Data fetched.")

#     while not rospy.is_shutdown():
       
#     # position = [[np.pi/2, np.pi/6, 0, 0, np.pi/2, 0, 0, 0],
#     #             [np.pi/2, np.pi/3, 0, 0, np.pi/2, 0, 0, 0], 
#     #             [np.pi/2, np.pi/2, 0, 0, np.pi/2, 0, 0, 0],
#     #             [np.pi/2, -np.pi/3, 0, 0, np.pi/2, 0, 0, 0],
#     #             [np.pi/2, -np.pi/6, 0, 0, np.pi/2, 0, 0, 0]]
    
#     # for i in range(4):
#     #     send_pose(desired_position=position[i])

    

#     rospy.loginfo("ROS node 'EKF' has been shutdown.")
#     # # reset_motors(motor_ID=[11, 12, 13, 14, 21, 22, 23, 24])  

# if __name__ == "__main__":
#     main()


def main():
    rospy.init_node('set_position_client_node')



    # positions = [np.pi/2, np.pi, np.pi/2, np.pi]  
    # send_pose(positions)


    position = [[np.pi/2, -np.pi/2, np.pi/2, 0],
                [np.pi/2, -np.pi/4, np.pi/2, 0], 
                [np.pi/2, 0, np.pi/2, 0],
                [np.pi/2, np.pi/4, np.pi/2, 0],
                [np.pi/2, np.pi/2, np.pi/2, 0]]
    
    for i in range(5):
        send_pose(positions = position[i])

    rospy.loginfo("Waiting before disabled")
    rospy.sleep(2)

    disable_motors()
    rospy.loginfo("Disabled")

    rospy.loginfo("Waiting before setting currents")
    rospy.sleep(2)

    currents = [100, 0, 0, 0]  
    send_current(currents)
    rospy.loginfo("Current Set")


    # rospy.spin()


if __name__ == '__main__':
    main()

'''
i do not understand what exactly is causing this problem:

when i run (in order)
    init_motors.py
    position_pub.py
    main_node.py

the nodes sometimes throws error while sometimes it works perfectly
'''