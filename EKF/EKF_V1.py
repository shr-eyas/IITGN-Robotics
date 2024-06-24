'''
Proprioception based EKF implementation

NOTE:
    The process model (state transition function) 'f' uses joint angles and velocities to predict the next state Y = [x1 y1 x2 y2]
    therefore, the jacobain matrix F = df/dY will be an indentity matrix. And, since the measurement model 'h' got the predicted measurements using joint angles which is not in the state Y, the 
    jacobian matrix H = dh/dY will also be an identity matrix. 
'''

import numpy as np
from numpy import sin, cos

def rotZ(theta):
    return np.array([
        [cos(theta), -sin(theta)],
        [sin(theta), cos(theta)]
    ])

def serialChainJacobian(l, phi):
    phi01 = phi[0] + phi[1] 
    return np.array([
        [(-l[0]*sin(phi[0]) - l[1]*sin(phi01)), -l[1]*sin(phi01)],
        [(l[0]*cos(phi[0]) + l[1]*cos(phi01)), l[1]*cos(phi01)]
    ])

def forwardKinematics(l, phi):
    phi01 = phi[0] + phi[1]
    x = l[0] * cos(phi[0]) + l[1] * cos(phi01)
    y = l[0] * sin(phi[0]) + l[1] * sin(phi01)
    return x, y

def stateTransitionFcn(Y, u, l1, l2, phi, shi):
    # Joint angles of the robot
    phi1 = phi[0]
    phi2 = phi[1]
    # Angle between palm frame and finger base frame 
    shi1 = shi[0]
    shi2 = shi[1]
    Rpk1 = rotZ(shi1)
    Rpk2 = rotZ(shi2)
    J1 = serialChainJacobian(l1, phi1)
    J2 = serialChainJacobian(l2, phi2)
    Y_pred = Y + np.vstack((np.hstack((Rpk1 @ J1, np.zeros(2,2))), np.hstack((np.zeros(2,2), Rpk2 @ J2)))) @ (u) 
    return Y_pred

def measurementFcn(l1, l2, phi):
    phi1 = phi[0]
    phi2 = phi[1]
    x1, y1 = forwardKinematics(l1, phi1)
    x2, y2 = forwardKinematics(l2, phi2)
    return np.array([[x1], [y1], [x2], [y2]])

def EKFPredict(Y, P, Q, u, l1, l2, phi, shi):
    Y_pred = stateTransitionFcn(Y, u, l1, l2, phi, shi)
    # df is linear wrt dy and therefore, df/dy = eye(4)
    F = np.eye(4)
    # Q is the covariance of the process noise 
    P_pred = F @ P @ F.T + Q
    # A-priori state estimate and error covariance calculated using system model 
    return Y_pred, P_pred

def EKFUpdate(Y_pred, P_pred, R, z, l1, l2, phi):
    # Note that multiple measurements can be added along with their covariance and sampling rate
    z_pred = measurementFcn(l1, l2, phi)
    H = np.eye(4)
    S = H @ P_pred @ H.T + R  # Residual covariance
    K = P_pred @ H.T @ np.linalg.inv(S)  # Kalman gain
    Y_updated = Y_pred + K @ (z - z_pred)
    # Where z is the actual measurement
    P_updated = (np.eye(len(K)) - K @ H) @ P_pred
    return Y_updated, P_updated

'''
INITIALIZATIONS
'''
Y0 = np.array([0.0, 0.0, 0.0, 0.0])  # [x1, y1, x2, y2]
P0 = np.diag([0.1, 0.1, 0.1, 0.1])  # Covariance for [x1, y1, x2, y2]

l1 = l2 = [0.103, 0.93]
shi = 0
num_steps = 1000

# Subscribe to joint angle publisher to fetch joint angles 'phi' and joint velocities 'phid'
u = None
phi = None

# Process noise covariance matrix 
Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance for [x1, y1, x2, y2]
# Measurement noise covariance matrix (example, diagonal matrix)
R = np.diag([0.1, 0.1, 0.1, 0.1])  # Measurement noise covariance for [x1, y1, x2, y2]
'''
Increase Q if the predicted state Y_pred significantly diverges from the actual measurements, indicating underestimation of process noise.
Increase R if the EKF estimates do not track sudden changes or measurements well, suggesting underestimation of measurement noise.
Decrease Q or R if the filter is overly sensitive to noise or exhibits excessive variability in state estimates.
'''
for t in range(num_steps):

    Y_pred, P_pred = EKFPredict(Y0, P0, Q, u, l1, l2, phi, shi)

    noise = np.random.normal(0, np.sqrt(R.diagonal()), size=4)  # Measurement noise
    z = measurementFcn(l1, l2, phi) + noise

    # Update step
    Y, P = EKFUpdate(Y_pred, P_pred, R, z, l1, l2, phi)

    print(f"Step {t+1}:")
    print("Predicted state (Y_pred):", Y_pred)
    print("Updated state (Y):", Y)
    print("Updated error covariance (P):")
    print(P)
    print()
