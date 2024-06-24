import numpy as np
import sympy as sp
from sympy import symbols 
import matplotlib.pyplot as plt
from scipy.optimize import minimize

class Utils:
    def __init__(self):
        self.mue_1 = 0.3  
        self.mue_2 = 0.3  

    def cross2D(self, v1, v2):
        return v1[0] * v2[1] - v1[1] * v2[0]

    def cube_dynamics(self, F, m, a, to, tf, dt):
        sigma_Fx = F[0]
        sigma_Fy = F[1]
        sigma_M = F[2]
        I = m * a ** 2 / 6
        g = 0
        W = m * g
        num_steps = len(np.arange(to, tf + dt, dt))
        Qdot = np.zeros((3, num_steps))
        Q = np.zeros((3, num_steps))
        timer = np.arange(to, tf + dt, dt)
        for i in range(len(timer) - 1):
            Qddot = [sigma_Fx[i] / m, (sigma_Fy[i] - W) / m, sigma_M[i] / I]
            Qdot[:, i + 1] = Qdot[:, i] + np.array(Qddot) * dt
            Q[:, i + 1] = Q[:, i] + Qdot[:, i + 1] * dt
        return Q, Qdot

    def grasp_matrix(self, theta, d):
        b1 = sp.Matrix([d, 0])
        b2 = sp.Matrix([-d, 0])
        R = self.rotZ(theta)
        R1 = self.rotZ(sp.pi/2 + theta)
        R2 = self.rotZ(3*sp.pi/2 + theta)
        G1 = sp.Matrix.vstack(R1, sp.Matrix([[self.cross2D(R * b1, R1.col(0)), self.cross2D(R * b1, R1.col(1))]]))
        G2 = sp.Matrix.vstack(R2, sp.Matrix([[self.cross2D(R * b2, R2.col(0)), self.cross2D(R * b2, R2.col(1))]]))
        G = sp.Matrix.hstack(G1, G2)
        return G

    def rotZ(self, theta):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)],
            [sp.sin(theta), sp.cos(theta)]
        ])

    # Define the SCMJacobian function
    def SCMJacobian(self, L, phi):
        phi_01 = phi[0] + phi[1]
        J = sp.Matrix([
            [-L[0] * sp.sin(phi[0]) - L[1] * sp.sin(phi_01),
            -L[1] * sp.sin(phi_01)],
            [L[0] * sp.cos(phi[0]) + L[1] * sp.cos(phi_01),
            L[1] * sp.cos(phi_01)]
        ])
        return J

    def hand_jacobian(self, phi1, phi2, L1, L2, theta, shi1, shi2):
        RPK1, RPK2 = self.rotZ(shi1), self.rotZ(shi2)
        R1, R2 = self.rotZ(sp.pi/2 + theta), self.rotZ(3*sp.pi/2 + theta)
        J1, J2 = self.SCMJacobian(L1, phi1), self.SCMJacobian(L2, phi2)
        Jh = sp.Matrix.vstack(
            sp.Matrix.hstack(R1.T * RPK1 * J1, sp.zeros(2, 2)),
            sp.Matrix.hstack(sp.zeros(2, 2), R2.T * RPK2 * J2)
        )
        return Jh

    def trajectory_planner(self, to, tf, thetai, thetadi, thetaf, thetadf):
        t = sp.symbols('t')
        Q = sp.Matrix([thetai, thetadi, thetaf, thetadf])
        t0 = to
        B = sp.Matrix([[1, t0, t0**2, t0**3],
                       [0, 1, 2*t0, 3*t0**2],
                       [1, tf, tf**2, tf**3],
                       [0, 1, 2*tf, 3*tf**2]])
        Binv1 = B.inv()
        A1 = Binv1 * Q
        a0 = A1[0]
        a1 = A1[1]
        a2 = A1[2]
        a3 = A1[3]
        theta_d = a0 + a1*t + a2*t**2 + a3*t**3
        thetadot_d = a1 + 2*a2*t + 3*a3*t**2
        thetaddot_d = 2*a2 + 6*a3*t
        return theta_d, thetadot_d, thetaddot_d

    def optimize_forces(self, F_x, F_y, M_z, a, initial_guess):
        """
        Optimize the forces given the constants and coefficients.

        Parameters:
        - F_x: Constant for x-direction force
        - F_y: Constant for y-direction force
        - M_z: Constant for moment
        - a: Coefficient related to moment arm
        - initial_guess: Initial guess for the optimization variables

        Returns:
        - Optimal value of the objective function
        - Optimal variables
        """
        
        mu = 0.3
        
        # Objective function: minimize f_1n + f_2n
        def objective(vars):
            f_1t, f_1n, f_2t, f_2n = vars
            return f_1n + f_2n

        # Equation constraints
        def eq_constraint1(vars):
            f_1t, f_1n, f_2t, f_2n = vars
            return f_1n - f_2n - F_x

        def eq_constraint2(vars):
            f_1t, f_1n, f_2t, f_2n = vars
            return f_2t - f_1t - F_y

        def eq_constraint3(vars):
            f_1t, f_1n, f_2t, f_2n = vars
            return (f_1t + f_2t) * a - M_z

        # Inequality constraints
        def ineq_constraint1(vars):
            f_1t, f_1n, f_2t, f_2n = vars
            return mu * f_1n - f_1t

        def ineq_constraint2(vars):
            f_1t, f_1n, f_2t, f_2n = vars
            return mu * f_2n - f_2t

        # Define the constraints in the form required by `minimize`
        cons = [
            {'type': 'eq', 'fun': eq_constraint1},
            {'type': 'eq', 'fun': eq_constraint2},
            {'type': 'eq', 'fun': eq_constraint3},
            {'type': 'ineq', 'fun': ineq_constraint1},
            {'type': 'ineq', 'fun': ineq_constraint2},
        ]

        # Perform the optimization
        result = minimize(objective, initial_guess, constraints=cons)

        return result.fun, result.x

    def optimal_finger_forces_(self, sigma_Fx, sigma_Fy, sigma_Mz, x0, a):

        def eq1(x):
            return x[1] - x[3] - sigma_Fx        
        def eq2(x):
            return x[2] - x[0] - sigma_Fy
        def eq3(x):
            return x[2] * a + x[0] * a - sigma_Mz
        def constraint1(x):
            return x[0] - self.mue_1 * x[1]
        def constraint2(x):
            return x[2] - self.mue_2 * x[3]
        def objective(x):
            return x[1] + x[3]
        def constraints(x):
            return [constraint1(x), constraint2(x)], [eq1(x), eq2(x), eq3(x)]
        
        res = minimize(objective, x0, constraints=({'type': 'ineq', 'fun': lambda x: constraints(x)[0]},
                                                {'type': 'eq', 'fun': lambda x: constraints(x)[1]}),
                    options={'disp': False})
        
        finger_forces = res.x
        
        return finger_forces
    
    def joint_torque(self):

        dt = 1e-2
        to = 0
        tf = 5
        m = 0.1 
        a = 0.1 
        timerx = np.arange(to, tf + dt, dt)

        t = symbols('t')
        yd, ydotd, yddotd = self.trajectory_planner(to, tf, 0, 0, 0.5, 0)
        xd, xdotd, xddotd = self.trajectory_planner(to, tf, 0, 0, 0, 0)
        th, thd, thdd = self.trajectory_planner(to, tf, 0, 0, 0, 0)
        
        y = np.array( [float(yd.subs(t, timer)) for timer in timerx] )
        x = np.array([float(xd.subs(t, timer)) for timer in timerx])
        theta = np.array([float(th.subs(t, timer)) for timer in timerx])
        
        trials = 50     
        lambda_ILC = 0.3
        gamma_ILC = 0.7

        # phi: vector of all finger joints 
        # psi: base frame to finger frame 

        # Desired trajectory vector (x, y, theta)
        Xd = np.vstack((x, y, theta))
        
        F = [np.zeros((3, len(timerx) - 1)) for _ in range(trials)]
        Er = [None] * trials
        G = [[None] * (len(timerx) - 1) for _ in range(trials)]
        Finger_F = [[np.zeros(4)] * (len(timerx) - 1) for _ in range(trials)]
        Tau = [[None] * (len(timerx) - 1) for _ in range(trials)]
        
        for i in range(trials):
            # Q is the actual trajectory vector (x, y, theta)
            Q, Qdot = self.cube_dynamics(F[i], m, a, to, tf, dt)
            
            for j in range(len(timerx) - 1):
                G[i][j] = self.grasp_matrix(Q[2, j], a)
                resultant_F = F[i]
                Fx_req = resultant_F[0, j]
                Fy_req = resultant_F[1, j]
                Mz_req = resultant_F[2, j]
                x0 = Finger_F[i][j-1] if j > 0 else [0, 0, 0, 0]
                Finger_F[i][j] = self.optimal_finger_forces(Fx_req, Fy_req, Mz_req, x0, a)

                # Jh = self.hand_jacobian([0, 0], [0, 0], [a, a], [a, a], Q[2, j], [0, 0], [0, 0])    
                # Jh_transpose = np.transpose(Jh)
                # Tau[i][j] = Jh_transpose.dot(Finger_F[i][j])
            
            Q_short = Q[:, :-1]
            e_x = Xd[0, :-1] - Q_short[0, :]
            e_y = Xd[1, :-1] - Q_short[1, :]
            e_theta = Xd[2, :-1] - Q_short[2, :]
            Er[i] = np.vstack((e_x, e_y, e_theta))
            if i < trials - 1:
                F[i + 1] = lambda_ILC * F[i] + gamma_ILC * Er[i]
    
        return Finger_F

    
    def IK_2R_elbow_up(self, X, Y, l1, l2):
        d = np.sqrt(X**2 + Y**2)
        calpha = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        salpha = np.sqrt(1 - calpha**2)
        alpha = np.arctan2(salpha, calpha)
        q2 = np.pi - alpha
        alp = np.arctan2(Y, X)
        beta = np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
        q1 = alp - beta
        return np.rad2deg(q1), np.rad2deg(q2)

    def IK_2R_elbow_down(self, X, Y, l1, l2):
        d = np.sqrt(X**2 + Y**2)
        calpha = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        salpha = np.sqrt(1 - calpha**2)
        alpha = np.arctan2(salpha, calpha)
        q2 = np.pi - alpha
        alp = np.arctan2(Y, X)
        calpha1 = (l1**2 + d**2 - l2**2) / (2 * l1 * d)
        salpha1 = np.sqrt(1 - calpha1**2)
        beta = np.arctan2(salpha1, calpha1)
        q1 = alp + beta
        q2 = -q2
        return np.rad2deg(q1), np.rad2deg(q2)


# utils = Grasp()
# q1, q2 = utils.IK_2R_elbow_up(-80, 137, 103, 88)
# q3, q4 = utils.IK_2R_elbow_down(80, 137, 103, 88)
# print([q1, q2, q3, q4])

# phi1 = [np.radians(90), np.radians(45)]
# phi2 = [np.radians(90), np.radians(-45)]
# L1 = [0.103, 0.093]
# L2 = [0.103, 0.093]
# theta = 0

# jh = utils.hand_jacobian(phi1, phi2, L1, L2, theta, 0, 0)
# print((np.array(jh)).shape)