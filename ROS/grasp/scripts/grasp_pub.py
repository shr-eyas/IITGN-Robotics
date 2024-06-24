#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import sympy as sp

def callback(data, publisher):   
    yaw = data.data[2]
    grasp_matrix = compute_grasp_matrix(yaw)
    publish_grasp_matrix(grasp_matrix, publisher)

def compute_grasp_matrix(yaw):
    theta = yaw  
    d = 0.0375
    b1 = sp.Matrix([d, 0])
    b2 = sp.Matrix([-d, 0])
    R = rotZ(theta)
    R1 = rotZ(sp.pi/2 + theta)
    R2 = rotZ(3*sp.pi/2 + theta)
    G1 = sp.Matrix.vstack(R1, sp.Matrix([[cross2D(R * b1, R1.col(0)), cross2D(R * b1, R1.col(1))]]))
    G2 = sp.Matrix.vstack(R2, sp.Matrix([[cross2D(R * b2, R2.col(0)), cross2D(R * b2, R2.col(1))]]))
    G = sp.Matrix.hstack(G1, G2)
    return G

def rotZ(angle):
        return sp.Matrix([[sp.cos(angle), -sp.sin(angle)], [sp.sin(angle), sp.cos(angle)]])

def cross2D(vec1, vec2):
    return vec1[0]*vec2[1] - vec1[1]*vec2[0]

def publish_grasp_matrix(grasp_matrix, publisher):
    flattened_matrix = grasp_matrix.tolist()
    flattened_list = [item for sublist in flattened_matrix for item in sublist]
    msg = Float64MultiArray(data=flattened_list)
    publisher.publish(msg)

def main():
    rospy.init_node('grasp_matrix_node', anonymous=True)
    publisher = rospy.Publisher('grasp_matrix', Float64MultiArray, queue_size=10)
    rospy.Subscriber('aruco_data', Float64MultiArray, callback, publisher)
    rospy.loginfo("Publishing Grasp Matrix...")

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()

