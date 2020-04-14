'''
Adapted from Extended Kalman Filter by Atsushi Sakai
//http://atsushiasakai.github.io/PythonRobotics

with state vector {x, y, heading, speed}
'''
import math
import numpy as np
import matplotlib.pyplot as plt


class Kalman_filter:
    def __init__(self):
        self.DT = 1
        self.t0 = 0
        self.ang = 0
        self.xEst = np.zeros((4,1))                        # state vector
        self.pEst = np.zeros((4,4))                        # state covariance matrix
        self.R = np.diag([1.0, 1.0])**2                    # process noise covariance  
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1])**2 # observation noise covariance

    # predict new position based on dead reconning
    # x - state vector input output
    # u - [speed, omega]
    def motion_model(self, x, u):
        print("motion x", x)
        print("motion u", u)
        F = np.array([[1.0, 0, 0, 0],             # state transition matrix
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 0]])
        B = np.array([[self.DT * math.cos(x[2, 0]), 0],   #N.B. x[2,0] = phi
                      [self.DT * math.sin(x[2, 0]), 0],
                      [0, self.DT],
                      [1.0, 0.0]])
#        print("motion B mat",B)
        x = F @ x + B @ u
#        print("motion x(t+1)", x)
        return x
    
    # x - state vector
    # z - state vector with observations
    def observation_model(self, x):
        H = np.array([                              # measurement function
            [1, 0, 0, 0],
            [0, 1, 0, 0]])
        z = H @ x
        return z

    #motion model Jacobian matrix
    # x - state vector
    # u - input vector
    def jacobiF(self, x, u):
        yaw = x[2, 0]
        v = u[0, 0]
        jF=np.array([
            [1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
            [0.0, 1.0, self.DT * v * math.cos(yaw) , self.DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])
        return jF

    #observation model Jacobian matrix
    def jacobiH(self):
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]])
        return jH

    # xEst - state vector
    # pEst - state transition matrix
    # z - GPS input
    # u speed/steering input
    def Kalman_update(self, z, u):
        # predict
        xPred = self.motion_model(self.xEst, u)
        jF = self.jacobiF(xPred, u)
        pPred = jF @ self.pEst @ jF.T + self.Q
        
        # update
        jH = self.jacobiH()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ pPred @ jH.T + self.R
        K = pPred @ jH.T @ np.linalg.inv(S)
#        print ("Kdoty ",K.dot(y))
        self.xEst = xPred + K @ y
        self.pEst = (np.eye(len(self.xEst)) - K @ jH) @ pPred
        return self.xEst


    # begin - start time of sample
    # v - fps
    # phi - heading radians
    # x - x-axis
    # y - y-axis
    def Kalman_start(self, begin, x, y, phi, v):
        self.t0 = begin
        self.oldphi = phi
        self.xEst = np.zeros((4,1))              # infinite a priori
        self.pEst = np.zeros((4,4))
        self.xEst[0, 0] = x
        self.xEst[1, 0] = y
        self.xEst[2, 0] = phi
        self.xEst[3, 0] = v

    # t - time of sample seconds
    # v - fps
    # phi = heading in radians (E, N)
    # x - x-axis
    # y - y-axis
    def Kalman_step(self, t, x, y, phi, v):
        self.DT = t - self.t0
        self.t0 = t
        self.omega = (phi - self.oldphi) / self.DT
        self.oldphi = phi
        u = np.array([[v], [self.omega]])
        z = np.array([[x], [y]])
        self.xEst = self.Kalman_update(z, u)
        pxy = self.pEst[0:2, 0:2]
        print("cov matrix ", self.pEst)
#        print("cov matrix ", pxy)
        eigval, eigvec = np.linalg.eig(pxy)
        print("eigs",eigval, eigvec)
        print("error ellipse",math.sqrt(eigval[0]),math.sqrt(eigval[1]))
        bigind = 0
        if eigval[0] < eigval[1]:
            bigind = 1
        angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
        print ("angle",(450 - math.degrees(angle))%360)
        return self.xEst

    def Kalman_state(self):
        return (self.pEst)
