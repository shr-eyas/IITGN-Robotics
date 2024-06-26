#!/usr/bin/env python3

import rospy
import numpy as np
from numpy import sqrt, pi, cos, sin, arctan2, array 

from std_msgs.msg import Float64MultiArray
from grasp.srv import GetPosition, SetPosition, SetPositionRequest, ResetMotor, ResetMotorRequest, SetCurrent, SetCurrentRequest

'''
FUNCTIONS TO HANDLE MOTORS
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
def get_motor_position():
    rospy.wait_for_service('get_position')
    try:
        get_position = rospy.ServiceProxy('get_position', GetPosition)
        response = get_position()
        return response.position
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return []
    
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
UTILITY FUNCTIONS
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
def grasp(a, theta):
    return array([[-sin(theta), -cos(theta), sin(theta), cos(theta)], 
        [cos(theta), -sin(theta), -cos(theta), sin(theta)], 
        [a*sin(theta)**2 + a*cos(theta)**2, 0, a*sin(theta)**2 + a*cos(theta)**2, 0]])
    
def handJacobian(theta, L, phi):
    link1 = L[0]
    link2 = L[1]
    q1 = phi[0]
    q2 = phi[1]
    q3 = phi[2]
    q4 = phi[3]
    return array([[-(-link1*sin(q1) - link2*sin(q1 + q2))*sin(theta) + (link1*cos(q1) + link2*cos(q1 + q2))*cos(theta), link2*sin(theta)*sin(q1 + q2) + link2*cos(theta)*cos(q1 + q2), 0, 0], 
        [-(-link1*sin(q1) - link2*sin(q1 + q2))*cos(theta) - (link1*cos(q1) + link2*cos(q1 + q2))*sin(theta), -link2*sin(theta)*cos(q1 + q2) + link2*sin(q1 + q2)*cos(theta), 0, 0], 
        [0, 0, (-link1*sin(q3) - link2*sin(q3 + q4))*sin(theta) - (link1*cos(q3) + link2*cos(q3 + q4))*cos(theta), -link2*sin(theta)*sin(q3 + q4) - link2*cos(theta)*cos(q3 + q4)], 
        [0, 0, (-link1*sin(q3) - link2*sin(q3 + q4))*cos(theta) + (link1*cos(q3) + link2*cos(q3 + q4))*sin(theta), link2*sin(theta)*cos(q3 + q4) - link2*sin(q3 + q4)*cos(theta)]])

def elbowUpIK(X, Y, l1, l2):
    d = sqrt(X**2 + Y**2)
    calpha = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
    salpha = sqrt(1 - calpha**2)
    alpha = arctan2(salpha, calpha)
    q2 = pi - alpha
    alp = arctan2(Y, X)
    beta = arctan2(l2 * sin(q2), l1 + l2 * cos(q2))
    q1 = alp - beta
    return q1, q2

def elbowDownIK(X, Y, l1, l2):
    d = sqrt(X**2 + Y**2)
    calpha = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
    salpha = sqrt(1 - calpha**2)
    alpha = arctan2(salpha, calpha)
    q2 = pi - alpha
    alp = arctan2(Y, X)
    calpha1 = (l1**2 + d**2 - l2**2) / (2 * l1 * d)
    salpha1 = sqrt(1 - calpha1**2)
    beta = arctan2(salpha1, calpha1)
    q1 = alp + beta
    q2 = -q2
    return q1, q2
    
'''
INITIALIZATIONS
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
dt = 1/1000.0  # Time step
T = 1 # Total time for each trajectory
t = np.arange(0, T, dt)
n_samples = int(T/dt)  # Number of samples in the trajectory
freq = 1/dt  # Frequency of control loop
trials = 100
Kt1 = 0.35
Kt2 = 0.51

a = 0.0375
L = [0.104, 0.085]

Kt1 = 0.35
Kt2 = 0.51

x1h, y1h = -80, 142
x2h, y2h = 80, 135
q1h, q2h =   elbowUpIK(x1h, y1h, L[0]*1000, L[1]*1000)
q3h, q4h = elbowDownIK(x2h, y2h, L[0]*1000, L[1]*1000)
homePosition = [q1h, q2h, q3h, q4h]

# The side length of the cube is set and used in Grasp Matrix and 
# Impedance Parameters
xDes = 0
yDes = 0.13
thetaDes = 0

KpX = 5000
KpY = 0
KpTheta = 50
Kp = array([[KpX, 0, 0], 
            [0, KpY, 0], 
            [0, 0, KpTheta]]) 

'''
DATA HANDLING
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
latest_data = {
    'object_position': None
    # 'motor_position': None
}
    
def aruco_callback(data):
    position = data.data 
    update_dictionary('object_position', position)

# def motor_callback(data):
#     motor_position = data.data 
#     update_dictionary('motor_position', motor_position)

def update_dictionary(key, value):
    latest_data[key] = value

'''
MAIN LOOP 
________________________________________________________________________________________________________________________________________________
'''
def ilc():
  
    # position = [[np.pi/2, -np.pi/2, np.pi/2, 0],
    #         [np.pi/2, -np.pi/4, np.pi/2, 0], 
    #         [np.pi/2, 0, np.pi/2, 0],
    #         [np.pi/2, np.pi/4, np.pi/2, 0],
    #         [np.pi/2, np.pi/2, np.pi/2, 0]]

    # for i in range(5):
    #     send_pose(positions = position[i])

    # rospy.loginfo("Waiting before disabled")
    # rospy.sleep(2)

    # disable_motors()
    # rospy.loginfo("Disabled")

    # rospy.loginfo("Waiting before setting currents")
    # rospy.sleep(2)

    # rospy.loginfo("Current Set")
    # currents = [100, 0, 0, 0]  
    # send_current(currents)
    # rospy.sleep(5)
    # disable_motors()

    # rospy.loginfo("Set Current tested. Testing subscription.")


    # while True:
    #     
    #     

    for i in range(trials):

        rospy.loginfo("Going to home position.")
        send_pose(homePosition)
        rospy.sleep(2)
        rospy.loginfo("At home position.")

        
        disable_motors()
        for j in range(len(t)):

            objectPosition = latest_data['object_position']
            # rospy.loginfo(f"Object Orientation{j}: {objectPosition[2]}")

            theta = objectPosition[2]
            G = grasp(a,theta)
            # rospy.loginfo(f"Grasp Matrix{j}: {graspMatrix}")

            motorPositions = get_motor_position()
            rospy.loginfo(f" Motor Position{j}: {motorPositions}")


            # motorPositions = latest_data['motor_position']
            
            Jh = handJacobian(theta, L, motorPositions)
            # rospy.loginfo(f"Hand Jacobian{j}: {Jh} \n {Jh}")

            f_null = (np.identity(4) - (np.linalg.pinv(G) @ G)) @ array([[500],[500],[500],[500]])
            
            rospy.loginfo(f"Motor Positions{j}: {f_null}")
            
            fingerF = f_null 

            JhT = np.transpose(Jh)

            tau = JhT @ fingerF

            current = [tau[0], tau[1], tau[2], tau[3]]
            current[0] = current[0]/Kt1
            current[1] = current[1]/Kt2
            current[2] = current[2]/Kt1
            current[3] = current[3]/Kt2

            
            send_current(current)

       
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
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
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

    rospy.init_node('ilc_node')

    rospy.on_shutdown(disable_motors)

    rospy.loginfo("Waiting for ArUco data")
    rospy.Subscriber('aruco_data', Float64MultiArray, aruco_callback)

    # '''
    # Experimental 
    # _____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
    # '''

    # rospy.Subscriber('motor_positions', Float64MultiArray, motor_callback)

    # '''
    # Experimental
    # _____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
    # '''

    while latest_data['object_position'] is None:
        rospy.sleep(0.1)

    rospy.loginfo("Fetched ArUco data") 

    ilc()
    
    rospy.spin()


if __name__ == '__main__':
    main()
