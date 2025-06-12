import numpy as np
import math
import time
from typing import Tuple

class EKFSensorFusion:
    def __init__(self, dt: float):
        # State vector: [x, y, phi, v] - hanya 4 state
        # x, y: posisi
        # phi: heading angle  
        # v: velocity
        self.state = np.zeros(4)
        self.P = np.eye(4) * 0.1  # Initial covariance lebih kecil
        self.dt = dt
        
        # More conservative Q (less confident in prediction)
        self.Q = np.diag([1.0, 1.0, 0.1, 1.0])  # 10x larger

        # More realistic input noise
        self.input_noise = np.diag([2.0, 0.5])  # Higher IMU noise

        # Keep R as is (GPS trust)
        self.R = np.diag([5.0, 5.0])
        
        # Tambahan untuk menyimpan K dan innovation
        self.K = None
        self.innovation = None
        
    def predict(self, imu_accel: float, imu_omega: float, dt: float = None) -> None:
        if dt is None:
            dt = self.dt
            
        x, y, phi, v = self.state
        
        # Motion model dengan IMU sebagai input
        # x_{k+1} = x_k + v_k * cos(phi_k) * dt
        # y_{k+1} = y_k + v_k * sin(phi_k) * dt  
        # phi_{k+1} = phi_k + omega * dt
        # v_{k+1} = v_k + accel * dt
        
        self.state = np.array([
            x + v * dt * math.cos(phi),
            y + v * dt * math.sin(phi),
            (phi + imu_omega * dt) % (2 * math.pi),
            v + imu_accel * dt
        ])
        
        # Jacobian matrix F (∂f/∂x)
        F = np.array([
            [1, 0, -v * dt * math.sin(phi), dt * math.cos(phi)],
            [0, 1,  v * dt * math.cos(phi), dt * math.sin(phi)],
            [0, 0,  1,                      0],
            [0, 0,  0,                      1]
        ])
        
        # Input Jacobian G (∂f/∂u) - bagaimana input noise mempengaruhi state
        G = np.array([
            [1, 0],  # x dari accel noise
            [1, 0],  # y dari accel noise
            [0,                           dt], # phi dari gyro noise
            [dt,                          0]   # v dari accel noise
        ])
        
        # Update covariance
        self.P = F @ self.P @ F.T + self.Q + G @ self.input_noise @ G.T
        
    def update(self, gps_measurement: np.ndarray) -> None:
        # H matrix: mapping dari state ke measurement
        H = np.array([
            [1, 0, 0, 0],  # GPS x
            [0, 1, 0, 0]   # GPS y
        ])
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain - simpan untuk akses nanti
        self.K = self.P @ H.T @ np.linalg.inv(S)
        
        # Innovation (measurement residual) - simpan untuk akses nanti
        predicted_measurement = H @ self.state
        self.innovation = gps_measurement - predicted_measurement
        
        # Update state and covariance
        self.state = self.state + self.K @ self.innovation
        self.P = (np.eye(4) - self.K @ H) @ self.P
        
    def get_position(self) -> Tuple[float, float]:
        return self.state[0], self.state[1]
        
    def get_heading(self) -> float:
        return self.state[2]
        
    def get_velocity(self) -> float:
        return self.state[3]
    
    # Getter methods baru
    def get_covariance(self) -> np.ndarray:
        return self.P.copy()
    
    def get_kalman_gain(self) -> np.ndarray:
        return self.K.copy() if self.K is not None else None
    
    def get_innovation(self) -> np.ndarray:
        return self.innovation.copy() if self.innovation is not None else None


# Contoh penggunaan dengan debugging
class EKFDebugger:
    def __init__(self, ekf: EKFSensorFusion):
        self.ekf = ekf
        self.history = []
        
    def step(self, imu_accel: float, imu_omega: float, gps_pos=None, dt=None):
        # Predict step
        state_before = self.ekf.state.copy()
        cov_before = np.trace(self.ekf.P)
        
        self.ekf.predict(imu_accel, imu_omega, dt)
        
        state_after_predict = self.ekf.state.copy()
        cov_after_predict = np.trace(self.ekf.P)
        
        # Update step (jika ada GPS)
        K_matrix = None
        innovation_vector = None
        if gps_pos is not None:
            self.ekf.update(gps_pos)
            K_matrix = self.ekf.get_kalman_gain()
            innovation_vector = self.ekf.get_innovation()
            
        state_final = self.ekf.state.copy()
        cov_final = np.trace(self.ekf.P)
        
        # Log untuk debugging
        step_info = {
            'state_before': state_before,
            'state_after_predict': state_after_predict,
            'state_final': state_final,
            'cov_before': cov_before,
            'cov_after_predict': cov_after_predict,
            'cov_final': cov_final,
            'imu_accel': imu_accel,
            'imu_omega': imu_omega,
            'gps_pos': gps_pos,
            'kalman_gain': K_matrix,
            'innovation': innovation_vector
        }
        
        self.history.append(step_info)
        
        # Warning jika covariance meledak
        if cov_final > 1000:
            print(f"WARNING: Covariance exploding! Trace = {cov_final:.2f}")
            
        return state_final


# Test sederhana
if __name__ == "__main__":
    # Inisialisasi
    dt = 0.1
    ekf = EKFSensorFusion(dt)
    debugger = EKFDebugger(ekf)
    
    # Set initial state
    ekf.state = np.array([0, 0, 0, 0])  # start at origin
    
    print("Testing EKF...")
    print("Initial state:", ekf.state)
    
    # Simulasi gerakan lurus dengan kecepatan konstan
    for i in range(10):
        # IMU data: accel = 0 (kecepatan konstan), omega = 0 (lurus)
        imu_accel = 0.0
        imu_omega = 0.0
        
        # GPS tersedia setiap 5 steps
        gps_pos = None
        if i % 5 == 0:
            # GPS dengan noise
            true_x = i * dt * 5  # assuming v=5 m/s
            true_y = 0
            gps_pos = np.array([true_x + np.random.normal(0, 1), 
                               true_y + np.random.normal(0, 1)])
        
        # Set initial velocity for this test
        if i == 0:
            ekf.state[3] = 5.0  # 5 m/s forward velocity
            
        state = debugger.step(imu_accel, imu_omega, gps_pos, dt)
        
        print(f"Step {i}: x={state[0]:.2f}, y={state[1]:.2f}, "
              f"phi={state[2]:.3f}, v={state[3]:.2f}")
        
        # Contoh akses P, K, dan innovation
        if gps_pos is not None:
            print(f"  Kalman gain: {ekf.get_kalman_gain()}")
            print(f"  Innovation: {ekf.get_innovation()}")
            print(f"  Covariance trace: {np.trace(ekf.get_covariance()):.2f}")
