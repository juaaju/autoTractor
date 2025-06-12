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
        P_diag = [15.0, 15.0, 1.0, 3.0, 0.2, 0.1]  # Increased initial uncertainty
        self.P = np.diag(P_diag)
        self.dt = dt
        
        # Process noise - disesuaikan untuk trajektori dengan turning tajam
        # Dinaikkan untuk memberikan lebih banyak trust pada prediction
        self.Q = np.diag([
            1.0,    # x process noise - dinaikkan untuk turning
            1.0,    # y process noise - dinaikkan untuk turning
            0.2,    # phi process noise - dinaikkan untuk sharp turns
            2.0,    # v process noise - dinaikkan untuk acceleration changes
            0.002,  # accel bias drift - sedikit dinaikkan
            0.002   # gyro bias drift - sedikit dinaikkan
        ])

        # Input noise - dinaikkan untuk lebih responsive
        self.input_noise = np.diag([2.0, 0.5])  # [accel_noise, gyro_noise]

        # Measurement noise - dinaikkan untuk kurangi trust pada GPS
        # GPS di lapangan terbuka dengan noise yang lebih realistis
        self.R = np.diag([10.0, 10.0])  # Reduced GPS trust for better IMU integration
        
        # Adaptive measurement noise parameters
        self.base_R = np.diag([10.0, 10.0])
        self.high_R = np.diag([25.0, 25.0])  # Higher noise during turns
        self.turn_threshold = 0.1  # rad/s threshold for detecting turns
        
        # Kalman gain dan innovation untuk debugging
        self.K = None
        self.innovation = None
        
        # Smoothing parameters - kurangi smoothing untuk lebih responsive
        self.velocity_alpha = 0.5   # Reduced smoothing untuk velocity
        self.heading_alpha = 0.6    # Reduced smoothing untuk heading
        self.prev_velocity = 0.0
        self.prev_heading = 0.0
        
        # Outlier detection - lebih permissive
        self.innovation_threshold = 25.0  # meter - increased threshold
        self.max_velocity = 10.0  # m/s - max speed untuk traktor
        self.max_accel = 5.0      # m/s² - increased max acceleration
        
        # Motion model improvements
        self.use_adaptive_noise = True
        self.turn_detection_window = []
        self.window_size = 5
        
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
    
    def _detect_turning(self, omega: float) -> bool:
        """Detect if vehicle is turning based on angular velocity"""
        self.turn_detection_window.append(abs(omega))
        if len(self.turn_detection_window) > self.window_size:
            self.turn_detection_window.pop(0)
        
        avg_omega = np.mean(self.turn_detection_window)
        return avg_omega > self.turn_threshold
    
    def _get_adaptive_R(self, omega: float) -> np.ndarray:
        """Get adaptive measurement noise based on turning state"""
        if not self.use_adaptive_noise:
            return self.R
        
        if self._detect_turning(omega):
            # During turns, trust IMU more than GPS
            return self.high_R
        else:
            # During straight motion, normal GPS trust
            return self.base_R
        
    def predict(self, imu_accel: float, imu_omega: float, dt: float = None) -> None:
        if dt is None:
            dt = self.dt
            
        # Clamp inputs untuk traktor - lebih permissive
        imu_accel = np.clip(imu_accel, -self.max_accel, self.max_accel)
        imu_omega = np.clip(imu_omega, -2.0, 2.0)  # Increased angular velocity limit
            
        x, y, phi, v, accel_bias, omega_bias = self.state
        
        # Corrected measurements
        corrected_accel = imu_accel - accel_bias
        corrected_omega = imu_omega - omega_bias
        
        # Improved motion model untuk traktor dua roda
        # Menggunakan bicycle model yang lebih akurat untuk turning
        new_v = v + corrected_accel * dt
        new_v = np.clip(new_v, -self.max_velocity, self.max_velocity)
        
        # Improved kinematic model untuk sharp turns
        if abs(corrected_omega) > 0.01:  # Non-zero angular velocity
            # Use exact bicycle model equations
            new_phi = phi + corrected_omega * dt
            new_phi = self._normalize_angle(new_phi)
            
            # More accurate position update for turning
            avg_phi = phi + corrected_omega * dt / 2.0
            avg_velocity = (v + new_v) / 2.0
            
            new_x = x + avg_velocity * dt * math.cos(avg_phi)
            new_y = y + avg_velocity * dt * math.sin(avg_phi)
        else:
            # Straight line motion
            new_phi = phi
            avg_velocity = (v + new_v) / 2.0
            new_x = x + avg_velocity * dt * math.cos(phi)
            new_y = y + avg_velocity * dt * math.sin(phi)
        
        # Bias models - random walk with slow drift
        new_accel_bias = accel_bias
        new_omega_bias = omega_bias
        
        self.state = np.array([new_x, new_y, new_phi, new_v, new_accel_bias, new_omega_bias])
        
        # Improved Jacobian matrix F (∂f/∂x) - 6x6
        avg_phi = phi + corrected_omega * dt / 2.0
        avg_velocity = (v + new_v) / 2.0
        cos_phi = math.cos(avg_phi)
        sin_phi = math.sin(avg_phi)
        
        F = np.array([
            [1, 0, -avg_velocity * dt * sin_phi, dt * cos_phi, 0, 0],
            [0, 1,  avg_velocity * dt * cos_phi, dt * sin_phi, 0, 0],
            [0, 0,  1,                           0,            0, -dt],
            [0, 0,  0,                           1,            -dt, 0],
            [0, 0,  0,                           0,            1, 0],
            [0, 0,  0,                           0,            0, 1]
        ])
        
        # Input Jacobian G (∂f/∂u) - 6x2
        G = np.array([
            [0.5 * dt**2 * cos_phi, -0.5 * avg_velocity * dt**2 * sin_phi],
            [0.5 * dt**2 * sin_phi,  0.5 * avg_velocity * dt**2 * cos_phi],
            [0,                      dt],
            [dt,                     0],
            [0,                      0],
            [0,                      0]
        ])
        
        # Adaptive process noise based on motion
        adaptive_Q = self.Q.copy()
        if abs(corrected_omega) > self.turn_threshold:
            # Increase process noise during turns
            adaptive_Q[0:3, 0:3] *= 2.0  # Increase position and angle uncertainty
        
        # Update covariance dengan numerical stability
        self.P = F @ self.P @ F.T + adaptive_Q + G @ self.input_noise @ G.T
        
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
        
        # Get adaptive measurement noise
        current_omega = len(self.turn_detection_window) > 0 and abs(self.turn_detection_window[-1]) or 0
        adaptive_R = self._get_adaptive_R(current_omega)
        
        # Innovation covariance
        S = H @ self.P @ H.T + adaptive_R
        
        # Ensure S is invertible
        if not self._is_positive_definite(S):
            S = self._make_positive_definite(S)
        
        # Kalman gain
        self.K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + self.K @ self.innovation
        
        # Joseph form update untuk numerical stability
        I_KH = np.eye(6) - self.K @ H
        self.P = I_KH @ self.P @ I_KH.T + self.K @ adaptive_R @ self.K.T
        
        # Normalize heading
        self.state[2] = self._normalize_angle(self.state[2])
        
        # Reduced velocity smoothing untuk lebih responsive
        self.state[3] = self.velocity_alpha * self.prev_velocity + (1 - self.velocity_alpha) * self.state[3]
        self.prev_velocity = self.state[3]
        
        return True
        
    def get_position(self) -> Tuple[float, float]:
        return self.state[0], self.state[1]
        
    def get_heading(self) -> float:
        # Reduced heading smoothing untuk lebih responsive
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
        if pos_uncertainty > 100.0:  # Increased threshold
            print("Filter unstable, resetting covariance...")
            P_diag = [15.0, 15.0, 1.0, 3.0, 0.2, 0.1]
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
        
        # Check stability dengan threshold yang lebih tinggi
        if uncertainty_final > 50.0:
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


# Test dengan simulasi traktor - improved test case
if __name__ == "__main__":
    print("Testing Improved EKF for Two-Wheel Tractor...")
    
    dt = 0.1
    ekf = EKFSensorFusion(dt)
    debugger = EKFDebugger(ekf)
    
    # Set initial state
    ekf.state = np.array([0, 0, 0, 0, 0.1, 0.05])  # dengan initial bias
    
    print("Initial state:", ekf.state)
    print("Improved parameters:")
    print(f"Process noise Q diagonal: {np.diag(ekf.Q)}")
    print(f"Measurement noise R diagonal: {np.diag(ekf.R)}")
    
    # Simulasi traktor bergerak dengan square trajectory
    for i in range(80):  # Extended simulation
        t = i * dt
        
        # Square trajectory simulation dengan sharp turns
        phase = (i // 20) % 4  # 4 phases: straight, turn, straight, turn
        
        if phase == 0:  # First straight
            imu_accel = 1.0 if i < 5 else 0.0  # Initial acceleration then constant
            imu_omega = 0.0
        elif phase == 1:  # First turn (90 degrees)
            imu_accel = -0.5  # Slight deceleration during turn
            imu_omega = 0.8   # Sharp turn
        elif phase == 2:  # Second straight
            imu_accel = 0.5 if i == 40 else 0.0  # Brief acceleration
            imu_omega = 0.0
        else:  # Second turn
            imu_accel = -0.5
            imu_omega = 0.8
        
        # Add realistic IMU noise
        imu_accel += np.random.normal(0, 0.1)
        imu_omega += np.random.normal(0, 0.05)
        
        # GPS dengan realistic rate dan noise
        gps_pos = None
        if i % 1 == 0:  # GPS every step untuk testing
            # Simulate noisy GPS measurements
            true_pos = debugger.ekf.get_position()
            gps_pos = np.array([
                true_pos[0] + np.random.normal(0, 3.0),  # GPS noise
                true_pos[1] + np.random.normal(0, 3.0)
            ])
        
        state = debugger.step(imu_accel, imu_omega, gps_pos, dt)
        
        if i % 10 == 0:
            print(f"\nStep {i} (Phase {phase}):")
            debugger.print_status()
            
    print("\nFinal improved test results:")
    debugger.print_status()
    print(f"Final position uncertainty: {debugger.ekf.get_position_uncertainty():.2f}m")
