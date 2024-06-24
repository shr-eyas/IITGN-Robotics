#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import sympy as sp

# Global variables
aruco_data = None
joint_angles = None
jacobian_publisher = None
rate = None

def aruco_callback(data):
    global aruco_data
    aruco_data = data.data
    compute_and_publish_jacobian()

def joint_callback(data):
    global joint_angles
    joint_angles = data.data
    compute_and_publish_jacobian()

def compute_and_publish_jacobian():

    global aruco_data, joint_angles, jacobian_publisher

    if aruco_data is None or joint_angles is None:
        return
    
    theta = aruco_data[2]  
    L1 = [0.104, 0.087]
    L2 = [0.104, 0.087]
    phi1 = [joint_angles[0], joint_angles[1]]
    phi2 = [joint_angles[2], joint_angles[3]]
    shi1 = 0
    shi2 = 0

    jh = hand_jacobian(phi1, phi2, L1, L2, theta, shi1, shi2)
    hand_jacobian_matrix = np.array(jh)
    flattened_matrix = hand_jacobian_matrix.flatten().tolist()

    msg = Float64MultiArray(data=flattened_matrix)
    jacobian_publisher.publish(msg)

def hand_jacobian(phi1, phi2, L1, L2, theta, shi1, shi2):    
    RPK1, RPK2 = rotZ(shi1), rotZ(shi2)
    R1, R2 = rotZ(sp.pi/2 + theta), rotZ(3*sp.pi/2 + theta)
    J1, J2 = SCMJacobian(L1, phi1), SCMJacobian(L2, phi2)
    Jh = sp.Matrix.vstack(
        sp.Matrix.hstack(R1.T * RPK1 * J1, sp.zeros(2, 2)),
        sp.Matrix.hstack(sp.zeros(2, 2), R2.T * RPK2 * J2)
    )
    return Jh

def rotZ(theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)],
        [sp.sin(theta), sp.cos(theta)]
    ])

def SCMJacobian(L, phi):
    phi_01 = phi[0] + phi[1]
    J = sp.Matrix([
        [-L[0] * sp.sin(phi[0]) - L[1] * sp.sin(phi_01),
        -L[1] * sp.sin(phi_01)],
        [L[0] * sp.cos(phi[0]) + L[1] * sp.cos(phi_01),
        L[1] * sp.cos(phi_01)]
    ])
    return J

def main():
    global jacobian_publisher, rate

    rospy.init_node('hand_jacobian_node', anonymous=True)
    jacobian_publisher = rospy.Publisher('hand_jacobian', Float64MultiArray, queue_size=10)
    rospy.Subscriber('aruco_data', Float64MultiArray, aruco_callback)
    rospy.Subscriber('motor_positions', Float64MultiArray, joint_callback)
    rospy.loginfo("Publishing Hand Jacobian...")
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
