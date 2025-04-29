import numpy as np
import math
import time
from typing import Tuple

class DummySensors:
    def __init__(self):
        self.t = 0
        self.true_x = 0
        self.true_y = 0
        self.true_heading = 0

    def get_gps_data(self) -> Tuple[float, float]:
        """Simulate GPS reading with some noise"""
        # Simulate circular motion
        self.true_x = 10 * math.cos(0.1 * self.t)
        self.true_y = 10 * math.sin(0.1 * self.t)

        # Add some noise
        gps_noise = np.random.normal(0, 0.5, 2)
        return (self.true_x + gps_noise[0],
                self.true_y + gps_noise[1])

    def get_imu_data(self) -> Tuple[float, float]:
        """Simulate IMU (acceleration and angular velocity) with noise"""
        # True values for circular motion
        true_omega = 0.1  # Angular velocity
        true_accel = -1.0 * (0.1**2) * 10  # Centripetal acceleration

        # Add noise
        imu_noise = np.random.normal(0, 0.01, 2)
        return (true_accel + imu_noise[0],  # acceleration
                true_omega + imu_noise[1])   # angular velocity

    def update(self):
        self.t += 0.1

class EKFSensorFusion:
    def __init__(self, dt: float):
        # State vector: [x, y, phi, omega, v, a]
        self.state = np.zeros(6)
        self.P = np.eye(6)  # State covariance
        self.dt = dt

        # Process noise
        self.Q = np.diag([0.1, 0.1])  # [angular acceleration noise, linear acceleration noise]
        # Process noise for all state variables
        # self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])  # [x, y, phi, omega, v, a]
        # Measurement noise
        self.R = np.diag([5.0, 5.0, 0.1, 0.1])  # [GPS_x, GPS_y, accel, gyro]

    def predict(self) -> None:
        """Prediction step of EKF"""
        x, y, phi, omega, v, a = self.state
        dt = self.dt
        
        # Predict next state
        self.state = np.array([
            x + v * dt * math.cos(phi),
            y + v * dt * math.sin(phi),
            phi + omega * dt,
            omega,  # assuming constant angular velocity
            v + a * dt,
            a      # assuming constant acceleration
        ])
        
        # Normalize heading to [0, 2π]
        self.state[2] = self.state[2] % (2 * math.pi)
        if self.state[2] < 0:
            self.state[2] += 2 * math.pi
        
        # Calculate Jacobian (phi matrix)
        phi_matrix = np.array([
            [1, 0, -math.sin(phi) * v * dt, v * dt, math.cos(phi) * dt, 0],
            [0, 1, math.cos(phi) * v * dt, 0, math.sin(phi) * dt, 0],
            [0, 0, 1, dt, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, dt],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # G matrix for process noise
        G = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [dt, 0],
            [0, 0],
            [0, dt]
        ])
        # G matrix becomes identity matrix when using full state noise model
        # G = np.eye(6)
        # Predict covariance
        self.P = phi_matrix @ self.P @ phi_matrix.T + G @ self.Q @ G.T

    def update(self, measurement: np.ndarray) -> None:
        """Update step of EKF using GPS and IMU measurements"""
        # Measurement matrix H
        # Maps state to measurements [x_gps, y_gps, accel, omega]
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # GPS x
            [0, 1, 0, 0, 0, 0],  # GPS y
            [0, 0, 0, 0, 0, 1],  # acceleration
            [0, 0, 0, 1, 0, 0]   # angular velocity
        ])

        # Calculate Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Calculate measurement residual
        predicted_measurement = H @ self.state
        y = measurement - predicted_measurement

        # Update state and covariance
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

def EKFtestingDummy():
    # Initialize
    dt = 0.1  # 10Hz update rate
    ekf = EKFSensorFusion(dt)
    sensors = DummySensors()

    # Storage for plotting
    n_steps = 200
    true_trajectory = np.zeros((n_steps, 2))
    estimated_trajectory = np.zeros((n_steps, 2))
    gps_measurements = np.zeros((n_steps, 2))

    try:
        for i in range(n_steps):  # Run for 20 seconds
            # Prediction step
            ekf.predict()

            # Get sensor measurements
            gps_x, gps_y = sensors.get_gps_data()
            accel, omega = sensors.get_imu_data()

            # Store true position and measurements
            true_trajectory[i] = [sensors.true_x, sensors.true_y]
            gps_measurements[i] = [gps_x, gps_y]

            # Update step with measurements
            measurements = np.array([gps_x, gps_y, accel, omega])
            
            ekf.update(measurements)

            # Store estimated position
            x, y = ekf.state[0:2]
            estimated_trajectory[i] = [x, y]

            # Update simulation time
            sensors.update()

            print(f"gps_x: {gps_x}, gps_y: {gps_y}, x: {x}, y: {y}")

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")

if __name__ == "__main__":
    EKFtestingDummy()