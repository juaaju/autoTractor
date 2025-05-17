import numpy as np
import math
import time
from typing import Tuple

class EKFSensorFusion:
    def __init__(self, dt: float):
        # State vector: [x, y, phi, omega, v, a]
        self.state = np.zeros(6)
        self.P = np.eye(6)  # State covariance
        self.dt = dt

        # Process noise
        #self.Q = np.diag([0.1, 0.1])  # [angular acceleration noise, linear acceleration noise]
        # Measurement noise
        # self.R = np.diag([5.0, 5.0, 0.1, 0.1])  # [GPS_x, GPS_y, accel, gyro]
        self.Q = np.diag([0.1, 0.1, 0.05, 0.01, 0.1, 0.05])  # Turunkan nilai Q

        # Kurang percaya pada GPS jika memang noise GPS signifikan
        self.R = np.diag([5.0, 5.0])  # Naikkan nilai R jika noise GPS besar

    # def predict(self) -> None:
    #     """Prediction step of EKF"""
    #     x, y, phi, omega, v, a = self.state
    #     dt = self.dt
        
    #     # Predict next state
    #     self.state = np.array([
    #         x + v * dt * math.cos(phi),
    #         y + v * dt * math.sin(phi),
    #         phi + omega * dt,
    #         omega,  # assuming constant angular velocity
    #         v + a * dt,
    #         a      # assuming constant acceleration
    #     ])
        
    #     # Normalize heading to [0, 2π]
    #     self.state[2] = self.state[2] % (2 * math.pi)
    #     if self.state[2] < 0:
    #         self.state[2] += 2 * math.pi
        
    #     # Calculate Jacobian (phi matrix)
    #     phi_matrix = np.array([
    #         [1, 0, -math.sin(phi) * v * dt, v * dt, math.cos(phi) * dt, 0],
    #         [0, 1, math.cos(phi) * v * dt, 0, math.sin(phi) * dt, 0],
    #         [0, 0, 1, dt, 0, 0],
    #         [0, 0, 0, 1, 0, 0],
    #         [0, 0, 0, 0, 1, dt],
    #         [0, 0, 0, 0, 0, 1]
    #     ])
        
    #     # G matrix for process noise
    #     G = np.array([
    #         [0, 0],
    #         [0, 0],
    #         [0, 0],
    #         [dt, 0],
    #         [0, 0],
    #         [0, dt]
    #     ])
    #     # G matrix becomes identity matrix when using full state noise model
    #     # G = np.eye(6)
    #     # Predict covariance
    #     self.P = phi_matrix @ self.P @ phi_matrix.T + G @ self.Q @ G.T

    def predict(self, imu_accel: float, imu_omega: float, dt: float = None) -> None:
        x, y, phi, omega, v, a = self.state
        # dt = self.dt
        dt = dt if dt is not None else self.dt
        # Gunakan imu_omega dan imu_accel sebagai input model untuk prediksi
        omega = imu_omega
        a = imu_accel

        self.state = np.array([
            x + v * dt * math.cos(phi),
            y + v * dt * math.sin(phi),
            (phi + omega * dt) % (2 * math.pi),
            omega,       # update omega dari IMU
            v + a * dt,  # update kecepatan dari percepatan IMU
            a           # update percepatan dari IMU
        ])

        # Jacobian tetap sama (kamu bisa sesuaikan jika mau model lebih rumit)
        phi_matrix = np.array([
            [1, 0, -math.sin(phi) * v * dt, v * dt, math.cos(phi) * dt, 0],
            [0, 1, math.cos(phi) * v * dt, 0, math.sin(phi) * dt, 0],
            [0, 0, 1, dt, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, dt],
            [0, 0, 0, 0, 0, 1]
        ])

        # # Process noise matrix sama, atau bisa kamu sesuaikan
        # G = np.array([
        #     [0, 0],
        #     [0, 0],
        #     [0, 0],
        #     [dt, 0],
        #     [0, 0],
        #     [0, dt]
        # ])
        G = np.eye(6)  # Full state noise model

        self.P = phi_matrix @ self.P @ phi_matrix.T + G @ self.Q @ G.T

    # def update(self, measurement: np.ndarray) -> None:
    #     """Update step of EKF using GPS and IMU measurements"""
    #     # Measurement matrix H
    #     # Maps state to measurements [x_gps, y_gps, accel, omega]
    #     H = np.array([
    #         [1, 0, 0, 0, 0, 0],  # GPS x
    #         [0, 1, 0, 0, 0, 0],  # GPS y
    #         [0, 0, 0, 0, 0, 1],  # acceleration
    #         [0, 0, 0, 1, 0, 0]   # angular velocity
    #     ])

    #     # Calculate Kalman gain
    #     S = H @ self.P @ H.T + self.R
    #     K = self.P @ H.T @ np.linalg.inv(S)

    #     # Calculate measurement residual
    #     predicted_measurement = H @ self.state
    #     y = measurement - predicted_measurement

    #     # Update state and covariance
    #     self.state = self.state + K @ y
    #     self.P = (np.eye(6) - K @ H) @ self.P

    def update(self, gps_measurement: np.ndarray) -> None:
        # H hanya untuk posisi x dan y
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # GPS x
            [0, 1, 0, 0, 0, 0]   # GPS y
        ])

        S = H @ self.P @ H.T + self.R[:2, :2]  # pakai R sesuai ukuran measurement GPS saja
        K = self.P @ H.T @ np.linalg.inv(S)

        predicted_measurement = H @ self.state
        y = gps_measurement - predicted_measurement

        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
