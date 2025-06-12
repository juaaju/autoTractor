import numpy as np
import math
import time
from typing import Tuple, Optional
from scipy.linalg import cholesky, LinAlgError

class EKFSensorFusion:
    def __init__(self, dt: float):
        # State vector: [x, y, phi, v, accel_bias, omega_bias] - 6 state untuk menangani bias
        # x, y: posisi
        # phi: heading angle  
        # v: velocity
        # accel_bias: bias akselerometer
        # omega_bias: bias gyroscope
        self.state = np.zeros(6)
        
        # Initial covariance - lebih konservatif untuk traktor
        P_diag = [10.0, 10.0, 0.5, 2.0, 0.1, 0.05]  # pos, pos, angle, vel, accel_bias, gyro_bias
        self.P = np.diag(P_diag)
        self.dt = dt
        
        # Process noise - disesuaikan untuk traktor dua roda yang lebih stabil
        # Traktor bergerak lebih lambat dan stabil dibanding mobil
        self.Q = np.diag([
            0.1,    # x process noise - traktor stabil
            0.1,    # y process noise  
            0.02,   # phi process noise - heading stabil
            0.5,    # v process noise - velocity bisa berubah
            0.001,  # accel bias drift - sangat kecil
            0.0005  # gyro bias drift - sangat kecil
        ])

        # Input noise - lebih realistis untuk IMU traktor
        self.input_noise = np.diag([1.5, 0.3])  # [accel_noise, gyro_noise]

        # Measurement noise - GPS untuk traktor di lapangan
        self.R = np.diag([3.0, 3.0])  # GPS accuracy di lapangan terbuka
        
        # Kalman gain dan innovation untuk debugging
        self.K = None
        self.innovation = None
        
        # Smoothing parameters
        self.velocity_alpha = 0.7  # Low-pass filter untuk velocity
        self.heading_alpha = 0.8   # Low-pass filter untuk heading
        self.prev_velocity = 0.0
        self.prev_heading = 0.0
        
        # Outlier detection
        self.innovation_threshold = 15.0  # meter - reject GPS outliers
        self.max_velocity = 10.0  # m/s - max speed untuk traktor
        self.max_accel = 3.0      # m/s² - max acceleration untuk traktor
        
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _is_positive_definite(self, matrix: np.ndarray) -> bool:
        """Check if matrix is positive definite"""
        try:
            cholesky(matrix)
            return True
        except LinAlgError:
            return False
    
    def _make_positive_definite(self, matrix: np.ndarray, min_eigenvalue: float = 1e-6) -> np.ndarray:
        """Make matrix positive definite by adding small values to diagonal"""
        eigenvals, eigenvecs = np.linalg.eigh(matrix)
        eigenvals = np.maximum(eigenvals, min_eigenvalue)
        return eigenvecs @ np.diag(eigenvals) @ eigenvecs.T
        
    def predict(self, imu_accel: float, imu_omega: float, dt: float = None) -> None:
        if dt is None:
            dt = self.dt
            
        # Clamp inputs untuk traktor
        imu_accel = np.clip(imu_accel, -self.max_accel, self.max_accel)
        imu_omega = np.clip(imu_omega, -1.0, 1.0)  # rad/s limit untuk traktor
            
        x, y, phi, v, accel_bias, omega_bias = self.state
        
        # Corrected measurements
        corrected_accel = imu_accel - accel_bias
        corrected_omega = imu_omega - omega_bias
        
        # Motion model untuk traktor dua roda
        # Menggunakan bicycle model yang disederhanakan
        new_v = v + corrected_accel * dt
        new_v = np.clip(new_v, -self.max_velocity, self.max_velocity)  # Limit velocity
        
        # Average velocity untuk smooth motion
        avg_velocity = (v + new_v) / 2.0
        
        new_phi = phi + corrected_omega * dt
        new_phi = self._normalize_angle(new_phi)
        
        new_x = x + avg_velocity * dt * math.cos(phi + corrected_omega * dt / 2.0)
        new_y = y + avg_velocity * dt * math.sin(phi + corrected_omega * dt / 2.0)
        
        # Bias models - random walk with very slow drift
        new_accel_bias = accel_bias  # konstant dengan noise di Q
        new_omega_bias = omega_bias  # konstant dengan noise di Q
        
        self.state = np.array([new_x, new_y, new_phi, new_v, new_accel_bias, new_omega_bias])
        
        # Jacobian matrix F (∂f/∂x) - 6x6
        cos_phi = math.cos(phi + corrected_omega * dt / 2.0)
        sin_phi = math.sin(phi + corrected_omega * dt / 2.0)
        
        F = np.array([
            [1, 0, -avg_velocity * dt * sin_phi, dt * cos_phi, 0, 0],
            [0, 1,  avg_velocity * dt * cos_phi, dt * sin_phi, 0, 0],
            [0, 0,  1,                          0,           0, -dt],
            [0, 0,  0,                          1,           -dt, 0],
            [0, 0,  0,                          0,           1, 0],
            [0, 0,  0,                          0,           0, 1]
        ])
        
        # Input Jacobian G (∂f/∂u) - 6x2
        G = np.array([
            [0.5 * dt**2 * cos_phi, 0],
            [0.5 * dt**2 * sin_phi, 0],
            [0,                     dt],
            [dt,                    0],
            [0,                     0],
            [0,                     0]
        ])
        
        # Update covariance dengan numerical stability
        self.P = F @ self.P @ F.T + self.Q + G @ self.input_noise @ G.T
        
        # Ensure P remains positive definite
        if not self._is_positive_definite(self.P):
            self.P = self._make_positive_definite(self.P)
            
    def update(self, gps_measurement: np.ndarray) -> bool:
        """Update dengan GPS measurement. Return True jika update berhasil."""
        # H matrix: mapping dari state ke measurement (hanya posisi)
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # GPS x
            [0, 1, 0, 0, 0, 0]   # GPS y
        ])
        
        # Predicted measurement
        predicted_measurement = H @ self.state
        self.innovation = gps_measurement - predicted_measurement
        
        # Outlier detection - reject GPS jika terlalu jauh
        innovation_magnitude = np.linalg.norm(self.innovation)
        if innovation_magnitude > self.innovation_threshold:
            print(f"GPS outlier rejected: innovation = {innovation_magnitude:.2f}m")
            return False
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Ensure S is invertible
        if not self._is_positive_definite(S):
            S = self._make_positive_definite(S)
        
        # Kalman gain
        self.K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + self.K @ self.innovation
        
        # Joseph form update untuk numerical stability
        I_KH = np.eye(6) - self.K @ H
        self.P = I_KH @ self.P @ I_KH.T + self.K @ self.R @ self.K.T
        
        # Normalize heading
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Velocity smoothing
        self.state[3] = self.velocity_alpha * self.prev_velocity + (1 - self.velocity_alpha) * self.state[3]
        self.prev_velocity = self.state[3]
        
        return True
        
    def get_position(self) -> Tuple[float, float]:
        return self.state[0], self.state[1]
        
    def get_heading(self) -> float:
        # Apply heading smoothing
        current_heading = self.state[2]
        smoothed_heading = self.heading_alpha * self.prev_heading + (1 - self.heading_alpha) * current_heading
        self.prev_heading = smoothed_heading
        return smoothed_heading
        
    def get_velocity(self) -> float:
        return self.state[3]
    
    def get_bias(self) -> Tuple[float, float]:
        """Return (accel_bias, gyro_bias)"""
        return self.state[4], self.state[5]
    
    def get_covariance(self) -> np.ndarray:
        return self.P.copy()
    
    def get_kalman_gain(self) -> Optional[np.ndarray]:
        return self.K.copy() if self.K is not None else None
    
    def get_innovation(self) -> Optional[np.ndarray]:
        return self.innovation.copy() if self.innovation is not None else None
    
    def get_position_uncertainty(self) -> float:
        """Return position uncertainty (standard deviation in meters)"""
        return math.sqrt(self.P[0,0] + self.P[1,1])
    
    def reset_if_unstable(self):
        """Reset filter jika tidak stabil"""
        pos_uncertainty = self.get_position_uncertainty()
        if pos_uncertainty > 50.0:  # 50 meter uncertainty terlalu besar
            print("Filter unstable, resetting covariance...")
            P_diag = [10.0, 10.0, 0.5, 2.0, 0.1, 0.05]
            self.P = np.diag(P_diag)


class EKFDebugger:
    def __init__(self, ekf: EKFSensorFusion):
        self.ekf = ekf
        self.history = []
        self.step_count = 0
        
    def step(self, imu_accel: float, imu_omega: float, gps_pos=None, dt=None):
        self.step_count += 1
        
        # Predict step
        state_before = self.ekf.state.copy()
        uncertainty_before = self.ekf.get_position_uncertainty()
        
        self.ekf.predict(imu_accel, imu_omega, dt)
        
        state_after_predict = self.ekf.state.copy()
        uncertainty_after_predict = self.ekf.get_position_uncertainty()
        
        # Update step (jika ada GPS)
        gps_update_success = False
        if gps_pos is not None:
            gps_update_success = self.ekf.update(gps_pos)
            
        state_final = self.ekf.state.copy()
        uncertainty_final = self.ekf.get_position_uncertainty()
        
        # Check stability
        if uncertainty_final > 30.0:
            self.ekf.reset_if_unstable()
        
        # Log untuk debugging
        step_info = {
            'step': self.step_count,
            'state_before': state_before,
            'state_after_predict': state_after_predict,
            'state_final': state_final,
            'uncertainty_before': uncertainty_before,
            'uncertainty_after_predict': uncertainty_after_predict,
            'uncertainty_final': uncertainty_final,
            'imu_accel': imu_accel,
            'imu_omega': imu_omega,
            'gps_pos': gps_pos,
            'gps_update_success': gps_update_success,
            'bias': self.ekf.get_bias()
        }
        
        self.history.append(step_info)
        
        return state_final
    
    def print_status(self):
        """Print current filter status"""
        x, y = self.ekf.get_position()
        heading = self.ekf.get_heading()
        velocity = self.ekf.get_velocity()
        accel_bias, gyro_bias = self.ekf.get_bias()
        uncertainty = self.ekf.get_position_uncertainty()
        
        print(f"Position: ({x:.2f}, {y:.2f}) ± {uncertainty:.2f}m")
        print(f"Heading: {math.degrees(heading):.1f}° | Velocity: {velocity:.2f} m/s")
        print(f"Bias - Accel: {accel_bias:.3f} | Gyro: {gyro_bias:.3f}")


# Test dengan simulasi traktor
if __name__ == "__main__":
    print("Testing Optimized EKF for Two-Wheel Tractor...")
    
    dt = 0.1
    ekf = EKFSensorFusion(dt)
    debugger = EKFDebugger(ekf)
    
    # Set initial state
    ekf.state = np.array([0, 0, 0, 0, 0.1, 0.05])  # dengan initial bias
    
    print("Initial state:", ekf.state)
    
    # Simulasi traktor bergerak dengan pola yang realistis
    for i in range(50):
        t = i * dt
        
        # Simulasi traktor bergerak dengan acceleration pattern yang smooth
        if i < 10:  # Acceleration phase
            imu_accel = 0.5 + np.random.normal(0, 0.1)  # accelerating
            imu_omega = 0.0 + np.random.normal(0, 0.05)  # straight
        elif i < 30:  # Constant speed
            imu_accel = 0.0 + np.random.normal(0, 0.1)  # constant speed
            imu_omega = 0.0 + np.random.normal(0, 0.05)  # straight
        elif i < 40:  # Turning
            imu_accel = -0.2 + np.random.normal(0, 0.1)  # slight deceleration
            imu_omega = 0.2 + np.random.normal(0, 0.05)  # turning
        else:  # Straight again
            imu_accel = 0.0 + np.random.normal(0, 0.1)
            imu_omega = 0.0 + np.random.normal(0, 0.05)
        
        # GPS tersedia dengan rate yang realistis (1Hz vs 10Hz IMU)
        gps_pos = None
        if i % 1 == 0:  # GPS setiap step untuk testing, biasanya lebih jarang
            # Simulate true position untuk comparison
            true_x = sum([debugger.history[j]['state_final'][3] * dt 
                         * math.cos(debugger.history[j]['state_final'][2]) 
                         for j in range(max(0, len(debugger.history)-5), len(debugger.history))])
            true_y = sum([debugger.history[j]['state_final'][3] * dt 
                         * math.sin(debugger.history[j]['state_final'][2]) 
                         for j in range(max(0, len(debugger.history)-5), len(debugger.history))])
            
            if i == 0:
                true_x, true_y = 0, 0
                
            # GPS dengan realistic noise
            gps_pos = np.array([true_x + np.random.normal(0, 2), 
                               true_y + np.random.normal(0, 2)])
        
        state = debugger.step(imu_accel, imu_omega, gps_pos, dt)
        
        if i % 10 == 0:
            print(f"\nStep {i}:")
            debugger.print_status()
            
    print("\nFinal test results:")
    debugger.print_status()
