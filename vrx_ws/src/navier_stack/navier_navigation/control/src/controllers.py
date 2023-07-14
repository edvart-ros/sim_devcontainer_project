#!/usr/bin/python3
import numpy as np
import time

class SurgePID():
    def __init__(self):
        return

class HeadingPID():
    def __init__(self):
        #PD controller gains and filter Constants 
        rate = 50
        self.Kp = 2
        self.Kd = 1.4
        self.T = 1/rate #time constants for LP-filter
        self.Tf = 0.02
        self.psi_d = 0
        self.force_d = 0
        self.psi_error = 0
        self.psi_error_prev = 0
        self.psi_derivative_prev = 0

        self.visualize_LOS = True

    def get_output(self):
        self.psi_error = np.arctan2(np.sin(self.psi_d-self.psi), np.cos(self.psi_d-self.psi)) #desired-actual angle error
        e_t, e_prev = self.psi_error, self.psi_error_prev
        Kp, Kd = self.Kp, self.Kd
        derivative = Kd*(e_t-e_prev)
        proportional = Kp*e_t
        
        #PD Control with Low Pass Filter
        torque_desired = proportional + derivative #(self.Tf/(self.Tf + self.T)) * self.psi_derivative_prev + (Kd/(self.Tf+self.T)) * (e_t - e_prev)
        self.psi_derivative_prev = derivative
        self.psi_error_prev = self.psi_error
        return torque_desired
    
    def set_state(self, psi_d, psi):
        self.psi_d = psi_d
        self.psi = psi
        return self.force_d
    
class DynamicPositioningPID():
    """3DOF (surge, sway, yaw) PID controller for dynamic positioning)"""
    def __init__(self, Kp=[0.2,0.7,0.3], Kd=[4, 0, 0], freq=50):
        rate = freq
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)
        self.x = None
        self.x_d = None
        self.error = np.zeros((3,1))
        self.x_prev = None
        self.time_prev = None

    def normalize_angle_diff(self, diff):
        while diff > np.pi:
            diff -= 2.0 * np.pi
        while diff <= -np.pi:
            diff += 2.0 * np.pi
        return diff

    def get_output(self):
        if self.x is None or self.x_d is None:
            raise Exception("Controller state or reference not set")
        if self.time_prev is None:
            self.time_prev = time.time()
        self.time_step = time.time() - self.time_prev
        self.error = self.x_d - self.x
        self.error[2, 0] = self.normalize_angle_diff(self.x_d[2, 0] - self.x[2, 0])

        if self.x_prev is not None:
            x_derivative = (self.x - self.x_prev)
            self.error_derivative = -x_derivative
        else:
            self.error_derivative = np.zeros_like(self.error)
        
        self.x_prev = self.x.copy()
        
        F = self.Kp @ self.error + self.Kd @ self.error_derivative
        R = np.array([[np.cos(self.x[2,0]), -np.sin(self.x[2,0]), 0],
                      [np.sin(self.x[2,0]),  np.cos(self.x[2,0]), 0],
                      [0,0,1]])
        u = R.T @ F
        if np.max(abs(u)) > 1:
            u = u/np.max(abs(u))
        self.time_prev = time.time()
        return u

    def set_state(self, x, y, psi):
        self.x = np.array([[x],[y],[psi]])
        return self.x

    def set_reference(self, x_d, y_d, psi_d):
        self.x_d = np.array([[x_d],[y_d],[psi_d]])
        return self.x_d
