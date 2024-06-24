#!/usr/bin/env python3

import rospy
import numpy as np
from numpy import sin, cos, arctan2
from std_msgs.msg import Float64MultiArray


# '''
# PROPRIOCEPTION EKF_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

# NOTE:
#     The process model (state transition function) 'f' uses joint angles and velocities to predict the next state Y = [x1 y1 x2 y2]
#     therefore, the jacobain matrix F = df/dY will be an indentity matrix. And, since the measurement model 'h' got the predicted 
#     measurements using joint angles which is not in the state Y, the jacobian matrix H = dh/dY will also be an identity matrix. 
# '''

# def stateTransitionFcn(Y, u, l1, l2, phi, shi):
#     # Joint angles of the robot
#     phi1 = phi[0]
#     phi2 = phi[1]
#     # Angle between palm frame and finger base frame 
#     shi1 = shi[0]
#     shi2 = shi[1]
#     Rpk1 = rotZ(shi1)
#     Rpk2 = rotZ(shi2)
#     J1 = serialChainJacobian(l1, phi1)
#     J2 = serialChainJacobian(l2, phi2)
#     Y_pred = Y + np.vstack((np.hstack((Rpk1 @ J1, np.zeros((2,2)))), np.hstack((np.zeros((2,2)), Rpk2 @ J2)))) @ (u) 
#     return Y_pred

# def measurementFcn(l1, l2, phi):
#     phi1 = phi[0]
#     phi2 = phi[1]
#     x1, y1 = forwardKinematics(l1, phi1)
#     x2, y2 = forwardKinematics(l2, phi2)
#     return np.array([[x1], [y1], [x2], [y2]])

# def rotZ(theta):
#     return np.array([
#         [cos(theta), -sin(theta)],
#         [sin(theta), cos(theta)]
#     ])

# def serialChainJacobian(l, phi):
#     phi01 = phi[0] + phi[1] 
#     return np.array([
#         [(-l[0]*sin(phi[0]) - l[1]*sin(phi01)), -l[1]*sin(phi01)],
#         [(l[0]*cos(phi[0]) + l[1]*cos(phi01)), l[1]*cos(phi01)]
#     ])

# def forwardKinematics(l, phi):
#     phi01 = phi[0] + phi[1]
#     x = l[0] * cos(phi[0]) + l[1] * cos(phi01)
#     y = l[0] * sin(phi[0]) + l[1] * sin(phi01)
#     return x, y

# def EKFPredict(Y, P, Q, u, l1, l2, phi, shi):
#     Y_pred = stateTransitionFcn(Y, u, l1, l2, phi, shi)
#     # df is linear wrt dy and therefore, df/dy = eye(4)
#     F = np.eye(4)
#     # Q is the covariance of the process noise 
#     P_pred = F @ P @ F.T + Q
#     # A-priori state estimate and error covariance calculated using system model 
#     return Y_pred, P_pred

# def EKFUpdate(Y_pred, P_pred, R, z, l1, l2, phi):
#     # Note that multiple measurements can be added along with their covariance and sampling rate
#     z_pred = measurementFcn(l1, l2, phi)
#     H = np.eye(4)
#     S = H @ P_pred @ H.T + R  # Residual covariance
#     K = P_pred @ H.T @ np.linalg.inv(S)  # Kalman gain
#     Y_updated = Y_pred + K @ (z - z_pred)
#     # Where z is the actual measurement
#     P_updated = (np.eye(len(K)) - K @ H) @ P_pred
#     return Y_updated, P_updated

# def EKF():

#     print("Waiting for data...")
#     while latest_data['joint_angles'] is None or latest_data['joint_velocity'] is None:
#         rospy.sleep(0.1)
      
#     u = latest_data['joint_velocity']
#     phi_ = latest_data['joint_angles']


#     phi_1 = [phi_[0], phi_[1]]
#     phi_2 = [phi_[2], phi_[3]]

#     phi = [phi_1, phi_2]

#     Y0 = np.array([0.0, 0.0, 0.0, 0.0])  # [x1, y1, x2, y2]
#     P0 = np.diag([1e-2, 1e-2, 1e-2, 1e-2])  # Covariance for [x1, y1, x2, y2]

#     l1 = l2 = [0.103, 0.93]
#     shi = [0, 0]

#     # Process noise covariance matrix 
#     Q = np.diag([1e-2, 1e-2, 1e-2, 1e-2])  # Process noise covariance for [x1, y1, x2, y2]
#     # Measurement noise covariance matrix (example, diagonal matrix)
#     R = np.diag([1e-2, 1e-2, 1e-2, 1e-2])  # Measurement noise covariance for [x1, y1, x2, y2]
#     '''
#     Increase Q if the predicted state Y_pred significantly diverges from the actual measurements, indicating underestimation of process noise.
#     Increase R if the EKF estimates do not track sudden changes or measurements well, suggesting underestimation of measurement noise.
#     Decrease Q or R if the filter is overly sensitive to noise or exhibits excessive variability in state estimates.
#     '''
#     while True:

#         Y_pred, P_pred = EKFPredict(Y0, P0, Q, u, l1, l2, phi, shi)

#         # noise = np.random.normal(0, np.sqrt(R.diagonal()), size=4)  # Measurement noise
#         z = measurementFcn(l1, l2, phi) #+ noise

#         # Update step
#         Y, P = EKFUpdate(Y_pred, P_pred, R, z, l1, l2, phi)

#         Y0 = Y
#         P0 = P

#         print("Predicted state (Y_pred):", Y_pred)
#         print("Updated state (Y):", Y)
#         print("Updated error covariance (P):")
#         print(P)
#         print()
#         rospy.sleep(1)
        
def EKF():
    
    def forwardKinematics(l, phi):
        phi01 = phi[0] + phi[1]
        x = l[0] * cos(phi[0]) + l[1] * cos(phi01)
        y = l[0] * sin(phi[0]) + l[1] * sin(phi01)
        return x, y
 
    phi_ = latest_data['joint_angles']

    phi_1 = [phi_[0], phi_[1]]
    phi_2 = [phi_[2], phi_[3]]

    
    l1 = l2 = [0.104, 0.087]
    # 10.4 cm = 104 mm = 0.104 m
    # 8.7 cm = 87 mm = 0.087 m
    # 2 mm uncertainity in measurement 0.002 
    x1, y1 = forwardKinematics(l1, phi_1)
    x2, y2 = forwardKinematics(l2, phi_2)

    

    x = (x1 + x2)/2
    y = (y1 + y2)/2

    theta = arctan2((y1-y2),(x1-x2))

    print([x*1000, y*1000, theta])


'''
DATA HANDLING
_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
'''
latest_data = {
    'object_position': None,
    'joint_angles': None,
    'joint_velocity': None
}
    
def aruco_callback(data):
    position = data.data 
    update_dictionary('object_position', position)

def motor_positions_callback(data):
    joint_angles = data.data
    update_dictionary('joint_angles', joint_angles)

def motor_velocities_callback(data):
    joint_velocity = data.data
    update_dictionary('joint_velocity', joint_velocity)

def update_dictionary(key, value):
    latest_data[key] = value

'''
MAIN LOOP 
________________________________________________________________________________________________________________________________________________
'''
def main():
    rospy.init_node('ekf', anonymous=True)

    rospy.Subscriber('aruco_data', Float64MultiArray, aruco_callback)
    rospy.Subscriber('motor_positions', Float64MultiArray, motor_positions_callback)
    rospy.Subscriber('motor_velocities', Float64MultiArray, motor_velocities_callback)

    print("ROS node 'EKF' is up.")
    print("Waiting for data.")

    while latest_data['joint_angles'] is None and latest_data['joint_velocity'] is None:
        rospy.sleep(0.1)

    print("Data fetched.")

    while not rospy.is_shutdown():
        EKF()
        rospy.sleep(1)


    print("ROS node 'EKF' has been shutdown.")

if __name__ == "__main__":
    main()
   
