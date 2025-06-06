import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock
from queue import Queue
from gpsread import GPSReader
from dfrobotimu import IMU_WT61PCTTL
import pymap3d as pm
from ekfnparam3 import EKFSensorFusion  # Import EKF Anda
from mpu9250read import mpu9250
from mpu6050read import mpu6050
from gpsimuhandler import GPSHandler, IMUHandler
import pymap3d as pm

def main_sensor_fusion():
    """Main sensor fusion dengan BENAR-BENAR terpisah IMU predict logging"""
    
    print("=== SENSOR FUSION WITH TRULY SEPARATE IMU PREDICT LOGGING ===")
    
    # Initialize hardware
    print("Initializing GPS...")
    gps_handler = GPSHandler()
    time.sleep(2)
    
    print("Initializing IMU...")
    imu_handler = IMUHandler()
    time.sleep(1)
    
    print("Initializing EKF...")
    dt = 0.05  # 20Hz
    
    # ✅ SOLUSI: DUA EKF INSTANCE TERPISAH!
    ekf_combined = EKFSensorFusion(dt=dt)  # Untuk combined fusion
    ekf_imu_only = EKFSensorFusion(dt=dt)  # Untuk IMU-only predict
    
    # Setup logging
    log_queue = Queue()  # Combined logging
    imu_predict_queue = Queue()  # IMU predict only logging
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f"sensor_fusion_{timestamp}.csv"
    imu_predict_filename = f"imu_predict_only_{timestamp}.csv"
    
    # Logging threads
    log_thread = Thread(target=logging_thread_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    imu_predict_thread = Thread(target=imu_predict_logging_func, args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    # Variables
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    predict_counter = 0
    
    print("Starting sensor fusion with truly separate dual logging...")
    print(f"Combined log: {log_filename}")
    print(f"IMU predict log: {imu_predict_filename}")
    
    try:
        iteration_count = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # Read IMU
            try:
                accel, gyro = imu_handler.get_data()
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
            
            # ✅ PREDICT PADA KEDUA EKF SECARA TERPISAH
            try:
                # Combined EKF predict (akan di-update GPS nanti)
                ekf_combined.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                # IMU-only EKF predict (TIDAK AKAN di-update GPS!)
                ekf_imu_only.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                predict_counter += 1
                
                # ✅ AMBIL PURE IMU PREDICT DARI EKF TERPISAH
                pure_predict_state = ekf_imu_only.state.copy()
                
                if len(pure_predict_state) == 6:
                    pure_pred_x, pure_pred_y, pure_pred_heading, pure_pred_omega, pure_pred_velocity, pure_pred_accel = pure_predict_state
                elif len(pure_predict_state) == 4:
                    pure_pred_x, pure_pred_y, pure_pred_heading, pure_pred_velocity = pure_predict_state
                    pure_pred_omega = gyro
                    pure_pred_accel = accel
                else:
                    print(f"⚠️ Unexpected EKF state dimension: {len(pure_predict_state)}")
                    break
                
                pure_pred_heading_deg = math.degrees(pure_pred_heading)
                
                # ✅ LOG PURE IMU PREDICT DATA
                imu_predict_data = [
                    current_time, dt_actual, accel, gyro, math.degrees(gyro),
                    pure_pred_x, pure_pred_y, pure_pred_heading_deg, pure_pred_velocity, pure_pred_accel,
                    predict_counter
                ]
                imu_predict_queue.put(imu_predict_data)
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                last_time = current_time
                continue
            
            # Read GPS
            gps_coords = gps_handler.get_coords()
            gps_valid = is_gps_valid(gps_coords)
            gps_updated = False
            
            # GPS initialization
            if gps_valid and not gps_initialized:
                lat_ref = gps_coords['latitude']
                lon_ref = gps_coords['longitude'] 
                alt_ref = gps_coords.get('altitude', 0)
                
                # Set initial position pada KEDUA EKF
                x_gps, y_gps = latlon_to_xy(lat_ref, lon_ref, alt_ref, 
                                           lat_ref, lon_ref, alt_ref)
                ekf_combined.state[0] = x_gps  # Should be 0
                ekf_combined.state[1] = y_gps  # Should be 0
                ekf_imu_only.state[0] = x_gps  # Should be 0  
                ekf_imu_only.state[1] = y_gps  # Should be 0
                
                gps_initialized = True
                print(f"GPS reference set: {lat_ref:.6f}, {lon_ref:.6f}")
            
            # ✅ GPS update HANYA pada combined EKF, IMU-only EKF tidak di-update!
            if gps_valid and gps_initialized:
                try:
                    gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                               gps_coords['longitude'],
                                               gps_coords.get('altitude', alt_ref),
                                               lat_ref, lon_ref, alt_ref)
                    
                    gps_measurement = np.array([gps_x, gps_y])
                    ekf_combined.update(gps_measurement)  # ⚠️ HANYA combined EKF!
                    gps_update_counter += 1
                    gps_updated = True
                except Exception as e:
                    print(f"GPS update error: {e}")
            
            # Extract final combined state
            if len(ekf_combined.state) == 6:
                final_est_x, final_est_y, final_est_heading, final_est_omega, final_est_velocity, final_est_accel = ekf_combined.state
            elif len(ekf_combined.state) == 4:
                final_est_x, final_est_y, final_est_heading, final_est_velocity = ekf_combined.state
                final_est_omega = gyro
                final_est_accel = accel
            else:
                print(f"⚠️ Unexpected EKF state dimension: {len(ekf_combined.state)}")
                break
                
            final_est_heading_deg = math.degrees(final_est_heading)
            
            # Combined logging
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                log_data = [
                    current_time, dt_actual,
                    gps_x, gps_y,
                    gps_coords['latitude'], gps_coords['longitude'],
                    accel, gyro, math.degrees(gyro),
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    gps_updated,
                    predict_counter, gps_update_counter
                ]
            else:
                log_data = [
                    current_time, dt_actual,
                    None, None, None, None,
                    accel, gyro, math.degrees(gyro),
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    False,
                    predict_counter, gps_update_counter
                ]
            
            log_queue.put(log_data)
            
            # Enhanced status print
            if iteration_count % 40 == 0:  # Every 2 seconds
                gps_stats = gps_handler.get_stats()
                imu_stats = imu_handler.get_stats()
                
                print(f"t={iteration_count*dt:.1f}s | "
                      f"GPS: {gps_stats['read_count']}/{gps_stats['error_count']} | "
                      f"IMU: {imu_stats['read_count']}/{imu_stats['error_count']} | "
                      f"Predicts: {predict_counter} | GPS updates: {gps_update_counter}")
                print(f"    IMU-only: ({pure_pred_x:.2f}, {pure_pred_y:.2f}) | "
                      f"Combined: ({final_est_x:.2f}, {final_est_y:.2f}) | "
                      f"Diff: ({abs(final_est_x-pure_pred_x):.2f}, {abs(final_est_y-pure_pred_y):.2f})")
                
                # Validasi: kedua hasil harus berbeda jika GPS aktif!
                if gps_initialized and abs(final_est_x - pure_pred_x) < 0.01 and abs(final_est_y - pure_pred_y) < 0.01:
                    print("⚠️ WARNING: IMU-only dan Combined state terlalu mirip! GPS mungkin tidak bekerja.")
            
            iteration_count += 1
            last_time = current_time
            
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Main loop error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        gps_handler.stop()
        imu_handler.stop()
        
        log_queue.put("STOP")
        imu_predict_queue.put("STOP")
        
        if log_thread.is_alive():
            log_thread.join(timeout=2)
        if imu_predict_thread.is_alive():
            imu_predict_thread.join(timeout=2)
            
        print(f"Combined data logged to: {log_filename}")
        print(f"IMU predict data logged to: {imu_predict_filename}")
        print("✅ TRULY FIXED: Sekarang menggunakan 2 EKF instance terpisah!")
        print("✅ IMU-only EKF tidak pernah di-update oleh GPS")
        print("✅ Combined EKF di-update oleh GPS")
        print("✅ Kedua log sekarang benar-benar independen!")

def is_gps_valid(gps_coords):
    if not gps_coords:
        return False
    
    lat = gps_coords.get('latitude', 0)
    lon = gps_coords.get('longitude', 0)
    
    if (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
        return False
    
    return True

def latlon_to_xy_simple(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Simple flat earth conversion"""
    R = 6371000  # Earth radius in meters
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat_ref_rad = math.radians(lat_ref)
    lon_ref_rad = math.radians(lon_ref)
    
    x = R * (lon_rad - lon_ref_rad) * math.cos(lat_ref_rad)
    y = R * (lat_rad - lat_ref_rad)
    
    return x, y

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """More accurate conversion using pymap3d"""
    x, y, z = pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)
    return x, y  # More accurate than flat earth
    
def logging_thread_func(q: Queue, filename: str):
    """Combined logging function"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel', 'gyro_rad', 'gyro_deg',
            'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
            'gps_updated', 'predict_count', 'gps_update_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def imu_predict_logging_func(q: Queue, filename: str):
    """IMU predict only logging function"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'accel', 'gyro_rad', 'gyro_deg',
            'predict_x', 'predict_y', 'predict_heading_deg', 
            'predict_velocity', 'predict_accel', 'predict_count'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def test_imu_predict_only():
    """Test khusus untuk melihat IMU predict tanpa GPS - CONTINUOUS VERSION"""
    print("=== Testing IMU Predict Only (No GPS Updates) - CONTINUOUS ===")
    
    print("Initializing IMU...")
    imu_handler = IMUHandler()
    time.sleep(1)
    
    print("Initializing EKF...")
    dt = 0.05
    ekf = EKFSensorFusion(dt=dt)
    
    # Set initial position manually
    ekf.state[0] = 0  # x
    ekf.state[1] = 0  # y
    
    # Setup logging
    imu_predict_queue = Queue()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    imu_predict_filename = f"imu_only_test_{timestamp}.csv"
    
    imu_predict_thread = Thread(target=imu_predict_logging_func, args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    print(f"IMU-only test logging to: {imu_predict_filename}")
    print("Press Ctrl+C to stop the test...")
    
    try:
        last_time = time.time()
        predict_counter = 0
        iteration_counter = 0  # Untuk tracking iterasi
        
        while True:  # INFINITE LOOP - berjalan sampai Ctrl+C
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # Read IMU
            try:
                accel, gyro = imu_handler.get_data()
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
            
            # EKF Predict only
            try:
                ekf.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                predict_counter += 1
                
                # Extract state
                if len(ekf.state) == 6:
                    pred_x, pred_y, pred_heading, pred_omega, pred_velocity, pred_accel = ekf.state
                elif len(ekf.state) == 4:
                    pred_x, pred_y, pred_heading, pred_velocity = ekf.state
                    pred_omega = gyro
                    pred_accel = accel
                else:
                    # Handle unexpected state dimensions
                    print(f"⚠️ Unexpected EKF state dimension: {len(ekf.state)}")
                    pred_x, pred_y = ekf.state[0], ekf.state[1]
                    pred_heading = ekf.state[2] if len(ekf.state) > 2 else 0
                    pred_velocity = ekf.state[3] if len(ekf.state) > 3 else 0
                    pred_omega = gyro
                    pred_accel = accel
                
                pred_heading_deg = math.degrees(pred_heading)
                
                # Log data
                imu_predict_data = [
                    current_time, dt_actual, accel, gyro, math.degrees(gyro),
                    pred_x, pred_y, pred_heading_deg, pred_velocity, pred_accel,
                    predict_counter
                ]
                imu_predict_queue.put(imu_predict_data)
                
                # Print status setiap 20 iterasi (sekitar 1 detik)
                if iteration_counter % 20 == 0:
                    elapsed_time = current_time - (current_time - iteration_counter * dt)
                    print(f"IMU #{iteration_counter}: "
                          f"Time: {iteration_counter * dt:.1f}s | "
                          f"Pos({pred_x:.2f}, {pred_y:.2f}) | "
                          f"V:{pred_velocity:.2f} | H:{pred_heading_deg:.1f}° | "
                          f"A:{accel:.3f} | Ω:{math.degrees(gyro):.1f}°/s")
                
                # Status laporan setiap 100 iterasi (sekitar 5 detik)
                if iteration_counter % 100 == 0 and iteration_counter > 0:
                    elapsed_minutes = (iteration_counter * dt) / 60
                    print(f"📊 Status: {iteration_counter} iterations, "
                          f"{elapsed_minutes:.1f} minutes elapsed, "
                          f"{predict_counter} predictions")
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                # Continue loop even if EKF fails
            
            iteration_counter += 1
            last_time = current_time
            
            # Timing
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print(f"\nTest stopped by user after {iteration_counter} iterations")
        print(f"Total time: {iteration_counter * dt:.1f} seconds ({(iteration_counter * dt)/60:.1f} minutes)")
    except Exception as e:
        print(f"\nUnexpected error after {iteration_counter} iterations: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        
        # Stop IMU handler
        try:
            imu_handler.stop()
            print("✓ IMU handler stopped")
        except Exception as e:
            print(f"Error stopping IMU handler: {e}")
        
        # Stop logging thread
        try:
            imu_predict_queue.put("STOP")
            if imu_predict_thread.is_alive():
                imu_predict_thread.join(timeout=5)
                if imu_predict_thread.is_alive():
                    print("⚠️ Warning: Logging thread did not terminate properly")
                else:
                    print("✓ Logging thread stopped")
        except Exception as e:
            print(f"Error stopping logging thread: {e}")
            
        print(f"✓ IMU-only test data logged to: {imu_predict_filename}")
        print(f"Final statistics:")
        print(f"  - Total iterations: {iteration_counter}")
        print(f"  - Total predictions: {predict_counter}")
        print(f"  - Total time: {iteration_counter * dt:.1f} seconds")

# Test functions (unchanged from your code)
def test_gps_only():
    print("=== Testing GPS Only ===")
    gps_handler = GPSHandler()
    
    try:
        for i in range(100):
            coords = gps_handler.get_coords()
            if coords:
                print(f"GPS #{i}: Lat: {coords['latitude']:.6f}, Lon: {coords['longitude']:.6f}")
            else:
                print(f"GPS #{i}: No data")
            time.sleep(0.5)
    finally:
        gps_handler.stop()

def test_imu_only():
    print("=== Testing IMU Only ===")
    imu_handler = IMUHandler()
    
    try:
        for i in range(100):
            accel, gyro = imu_handler.get_data()
            print(f"IMU #{i}: Accel: {accel:.3f}, Gyro: {math.degrees(gyro):.3f}°/s")
            time.sleep(0.1)
    finally:
        imu_handler.stop()

def test_ekf_dimensions():
    """Test untuk cek dimensi EKF state vector"""
    print("=== Testing EKF Dimensions ===")
    ekf = EKFSensorFusion(dt=0.1)
    
    print(f"EKF state vector dimension: {len(ekf.state)}")
    print(f"EKF state vector: {ekf.state}")
    
    # Test predict
    ekf.predict(1.0, 0.1)
    print(f"After predict: {ekf.state}")
    
    if len(ekf.state) == 6:
        print("✓ Using 6D state vector [x, y, phi, omega, v, a]")
    elif len(ekf.state) == 4:
        print("✓ Using 4D state vector [x, y, phi, v]")
    else:
        print(f"⚠️ Unexpected dimension: {len(ekf.state)}")

def main():
    print("GPS-IMU Sensor Fusion Debug Tool")
    print("1. Test GPS only")
    print("2. Test IMU only") 
    print("3. Test EKF dimensions")
    print("4. Run main sensor fusion (with dual logging)")
    print("5. Test IMU predict only (no GPS updates)")
    
    choice = input("Enter choice (1-5): ").strip()
    
    if choice == '1':
        test_gps_only()
    elif choice == '2':
        test_imu_only()
    elif choice == '3':
        test_ekf_dimensions()
    elif choice == '4':
        main_sensor_fusion()
    elif choice == '5':
        test_imu_predict_only()
    else:
        print("Invalid choice")

if __name__ == "__main__":
    main()
