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

        # Process noise - untuk input IMU [linear_accel_noise, angular_vel_noise]
        self.Q = np.diag([0.5, 0.5])  # [accel_noise_std, gyro_noise_std]
        
        # Measurement noise - untuk GPS [x_noise, y_noise]
        self.R = np.diag([10.0, 10.0])  # [GPS_x_noise, GPS_y_noise]

    def predict(self, imu_accel: float, imu_omega: float, dt: float = None) -> None:
        x, y, phi, omega, v, a = self.state
        dt = dt if dt is not None else self.dt
        
        # Simpan nilai lama untuk Jacobian
        phi_old = phi
        v_old = v
        
        # Update state menggunakan IMU sebagai input
        self.state = np.array([
            x + v_old * dt * math.cos(phi_old),
            y + v_old * dt * math.sin(phi_old),
            (phi_old + imu_omega * dt) % (2 * math.pi),
            imu_omega,           # omega langsung dari IMU
            v_old + imu_accel * dt,  # velocity update dari accel
            imu_accel            # acceleration langsung dari IMU
        ])

        # Jacobian terhadap state (menggunakan nilai lama)
        phi_matrix = np.array([
            [1, 0, -math.sin(phi_old) * v_old * dt, 0, math.cos(phi_old) * dt, 0],
            [0, 1, math.cos(phi_old) * v_old * dt, 0, math.sin(phi_old) * dt, 0],
            [0, 0, 1, dt, 0, 0],
            [0, 0, 0, 0, 0, 0],  # omega tidak bergantung pada state lama (langsung dari IMU)
            [0, 0, 0, 0, 1, 0],  # velocity tidak bergantung pada omega lama
            [0, 0, 0, 0, 0, 0]   # acceleration tidak bergantung pada state lama (langsung dari IMU)
        ])

        # G matrix: bagaimana noise input IMU mempengaruhi state
        G = np.array([
            [0.5 * dt**2 * math.cos(phi_old), 0],  # posisi x dari accel noise
            [0.5 * dt**2 * math.sin(phi_old), 0],  # posisi y dari accel noise  
            [0,                               dt], # phi dari gyro noise
            [0,                               1],  # omega langsung dari gyro noise
            [dt,                              0],  # velocity dari accel noise
            [1,                               0]   # acceleration langsung dari accel noise
        ])

        # Update covariance
        self.P = phi_matrix @ self.P @ phi_matrix.T + G @ self.Q @ G.T

    def update(self, gps_measurement: np.ndarray) -> None:
        # H matrix: mapping dari state ke measurement
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # GPS x
            [0, 1, 0, 0, 0, 0]   # GPS y
        ])

        # Kalman gain calculation
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Innovation (measurement residual)
        predicted_measurement = H @ self.state
        y = gps_measurement - predicted_measurement

        # Update state and covariance
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P