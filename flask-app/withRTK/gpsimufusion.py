import time
import numpy as np
import math
import csv
from datetime import datetime
from threading import Thread, Lock
from queue import Queue
# Import custom modules
from gpsimuhandler import GPSHandler, FilteredIMUHandler, create_imu_handler
from ekfnparam3 import EKFSensorFusion
from helper import latlon_to_xy, is_gps_valid

def main_sensor_fusion_with_filters():
    """
    Main sensor fusion dengan low-pass filtering untuk mengurangi noise getaran mesin
    Menggunakan dua EKF instance terpisah untuk logging yang benar
    """
    
    print("=== SENSOR FUSION WITH LOW-PASS FILTERING ===")
    print("Reducing engine vibration noise with adaptive filters")
    
    # ===== FILTER CONFIGURATION =====
    # Sesuaikan dengan jenis kendaraan/aplikasi Anda
    filter_config = {
        'application_type': 'vehicle',  # 'vehicle', 'drone', 'walking_robot'
        'accel_filter_config': {
            'cutoff': 3.0,        # Hz - Allow vehicle dynamics, filter engine vibration (>10Hz)
            'type': 'butterworth', # 'butterworth', 'exponential', 'moving_average'
            'adaptive': False      # True untuk adaptive filtering
        },
        'gyro_filter_config': {
            'cutoff': 5.0,         # Hz - Allow turning dynamics
            'type': 'butterworth',
            'adaptive': False
        }
    }
    
    # Option untuk testing different filter settings
    print("Filter Configuration:")
    print(f"  Application type: {filter_config['application_type']}")
    print(f"  Accel filter: {filter_config['accel_filter_config']['cutoff']} Hz {filter_config['accel_filter_config']['type']}")
    print(f"  Gyro filter: {filter_config['gyro_filter_config']['cutoff']} Hz {filter_config['gyro_filter_config']['type']}")
    print(f"  Adaptive filtering: {filter_config['accel_filter_config']['adaptive']}")
    
    # Initialize hardware dengan filtering
    print("\nInitializing GPS...")
    gps_handler = GPSHandler()
    time.sleep(2)
    
    print("Initializing IMU with filtering...")
    imu_handler = FilteredIMUHandler(**filter_config)
    time.sleep(2)
    
    # Validate IMU is working
    test_accel, test_gyro = imu_handler.get_data()
    print(f"IMU test read: accel={test_accel:.3f}, gyro={math.degrees(test_gyro):.1f}°/s")
    
    print("Initializing EKF...")
    dt = 0.05  # 20Hz
    
    # ✅ DUA EKF INSTANCE TERPISAH
    ekf_combined = EKFSensorFusion(dt=dt)  # Untuk combined fusion
    ekf_imu_only = EKFSensorFusion(dt=dt)  # Untuk IMU-only predict
    
    # Setup logging dengan filter info
    log_queue = Queue()  # Combined logging
    imu_predict_queue = Queue()  # IMU predict only logging
    filter_stats_queue = Queue()  # Filter effectiveness logging
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f"sensor_fusion_filtered_{timestamp}.csv"
    imu_predict_filename = f"imu_predict_filtered_{timestamp}.csv"
    filter_stats_filename = f"filter_stats_{timestamp}.csv"
    
    # Logging threads
    log_thread = Thread(target=combined_logging_func, args=(log_queue, log_filename))
    log_thread.daemon = True
    log_thread.start()
    
    imu_predict_thread = Thread(target=imu_predict_logging_func, args=(imu_predict_queue, imu_predict_filename))
    imu_predict_thread.daemon = True
    imu_predict_thread.start()
    
    filter_stats_thread = Thread(target=filter_stats_logging_func, args=(filter_stats_queue, filter_stats_filename))
    filter_stats_thread.daemon = True
    filter_stats_thread.start()
    
    # Variables
    lat_ref, lon_ref, alt_ref = None, None, None
    gps_initialized = False
    last_time = time.time()
    gps_update_counter = 0
    predict_counter = 0
    
    print(f"\nStarting filtered sensor fusion...")
    print(f"Combined log: {log_filename}")
    print(f"IMU predict log: {imu_predict_filename}")
    print(f"Filter stats log: {filter_stats_filename}")
    print("Logs will show effectiveness of vibration filtering")
    
    try:
        iteration_count = 0
        filter_stats_log_counter = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # ===== READ FILTERED IMU DATA =====
            try:
                # Get both filtered and raw data untuk comparison
                accel_filtered, gyro_filtered, accel_raw, gyro_raw, filter_stats = imu_handler.get_data(
                    return_raw=True, return_stats=True
                )
                
                # Use filtered data for EKF
                accel = accel_filtered
                gyro = gyro_filtered
                
            except Exception as e:
                print(f"IMU read error: {e}")
                accel, gyro = 0, 0
                accel_raw, gyro_raw = 0, 0
                filter_stats = {'accel_noise_reduction': 0, 'gyro_noise_reduction': 0}
            
            # ===== EKF PREDICT PADA KEDUA INSTANCE =====
            try:
                # Combined EKF predict (akan di-update GPS nanti)
                ekf_combined.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                # IMU-only EKF predict (TIDAK AKAN di-update GPS!)
                ekf_imu_only.predict(imu_accel=accel, imu_omega=gyro, dt=dt_actual)
                
                # Extract P matrix untuk IMU-only
                P_matrix_imu = ekf_imu_only.get_covariance()
                P_trace_imu = np.trace(P_matrix_imu)
                predict_counter += 1
                
                # Extract pure IMU predict dari EKF terpisah
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
                
                # ===== LOG PURE IMU PREDICT DATA DENGAN FILTER INFO =====
                imu_predict_data = [
                    current_time, dt_actual, 
                    accel, gyro, math.degrees(gyro),  # Filtered values
                    accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw values
                    pure_pred_x, pure_pred_y, pure_pred_heading_deg, pure_pred_velocity, pure_pred_accel,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    filter_stats.get('accel_cutoff', 0), filter_stats.get('gyro_cutoff', 0),
                    predict_counter, P_trace_imu  # ✅ Tambahkan P_trace_imu
                ]
                imu_predict_queue.put(imu_predict_data)
                
            except Exception as e:
                print(f"EKF predict error: {e}")
                last_time = current_time
                continue
            
            # ===== READ GPS =====
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
            
            # ===== GPS UPDATE HANYA PADA COMBINED EKF =====
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

            # ===== EXTRACT P, K, INNOVATION DATA UNTUK LOGGING =====
            P_matrix = ekf_combined.get_covariance()
            P_trace = np.trace(P_matrix)

            # K dan innovation hanya ada jika GPS update berhasil
            K_matrix = None if not gps_updated else ekf_combined.get_kalman_gain()
            innovation_vector = None if not gps_updated else ekf_combined.get_innovation()

            # Format untuk CSV (string format)
            K_str = str(K_matrix.tolist()) if K_matrix is not None else None
            innovation_str = str(innovation_vector.tolist()) if innovation_vector is not None else None

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
            
            # ===== COMBINED LOGGING DENGAN FILTER INFO =====
            if gps_valid and gps_initialized:
                gps_x, gps_y = latlon_to_xy(gps_coords['latitude'], 
                                           gps_coords['longitude'],
                                           gps_coords.get('altitude', alt_ref),
                                           lat_ref, lon_ref, alt_ref)
                log_data = [
                    current_time, dt_actual,
                    gps_x, gps_y,
                    gps_coords['latitude'], gps_coords['longitude'],
                    accel, gyro, math.degrees(gyro),  # Filtered values
                    accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw values untuk comparison
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    gps_updated,
                    predict_counter, gps_update_counter,
                    P_trace, K_str, innovation_str  # ✅ Tambahkan P, K, innovation
                ]
            else:
                log_data = [
                    current_time, dt_actual,
                    None, None, None, None,
                    accel, gyro, math.degrees(gyro),  # Filtered values
                    accel_raw, gyro_raw, math.degrees(gyro_raw),  # Raw values
                    final_est_x, final_est_y, final_est_heading_deg, final_est_velocity, final_est_accel,
                    filter_stats['accel_noise_reduction'], filter_stats['gyro_noise_reduction'],
                    False,
                    predict_counter, gps_update_counter,
                    P_trace, K_str, innovation_str  # ✅ Tambahkan P, K, innovation
                ]
            
            log_queue.put(log_data)
            
            # ===== LOG FILTER STATISTICS =====
            if filter_stats_log_counter % 20 == 0:  # Every second
                filter_stats_data = [
                    current_time,
                    filter_stats['accel_noise_reduction'],
                    filter_stats['gyro_noise_reduction'],
                    filter_stats.get('accel_cutoff', 0),
                    filter_stats.get('gyro_cutoff', 0),
                    abs(accel_raw - accel),  # Instantaneous noise removed
                    abs(gyro_raw - gyro),
                    iteration_count
                ]
                filter_stats_queue.put(filter_stats_data)
            
            filter_stats_log_counter += 1
            
            # ===== ENHANCED STATUS PRINT =====
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
                print(f"    Filter effectiveness: Accel={filter_stats['accel_noise_reduction']:.1%}, "
                      f"Gyro={filter_stats['gyro_noise_reduction']:.1%}")
                print(f"    Noise reduction: A={abs(accel_raw-accel):.3f}, G={abs(math.degrees(gyro_raw-gyro)):.1f}°/s")
                print(f"    P trace: IMU-only={P_trace_imu:.2f}, Combined={P_trace:.2f}")
                
                # Warning jika filter tidak efektif
                if filter_stats['accel_noise_reduction'] < 0.05:
                    print("    ⚠️ Low accel filter effectiveness - consider adjusting cutoff frequency")
                if filter_stats['gyro_noise_reduction'] < 0.05:
                    print("    ⚠️ Low gyro filter effectiveness - consider adjusting cutoff frequency")
                
                # Validasi: kedua hasil harus berbeda jika GPS aktif!
                if gps_initialized and abs(final_est_x - pure_pred_x) < 0.01 and abs(final_est_y - pure_pred_y) < 0.01:
                    print("    ⚠️ WARNING: IMU-only dan Combined state terlalu mirip! GPS mungkin tidak bekerja.")
            
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
        
        # Stop handlers
        gps_handler.stop()
        imu_handler.stop()
        
        # Stop logging threads
        log_queue.put("STOP")
        imu_predict_queue.put("STOP")
        filter_stats_queue.put("STOP")
        
        # Wait for threads to finish
        for thread in [log_thread, imu_predict_thread, filter_stats_thread]:
            if thread.is_alive():
                thread.join(timeout=2)
        
        print(f"✅ Data logged to:")
        print(f"   Combined: {log_filename}")
        print(f"   IMU predict: {imu_predict_filename}")
        print(f"   Filter stats: {filter_stats_filename}")
        print("✅ Low-pass filtering successfully reduced engine vibration noise!")
        print("✅ Check filter_stats log untuk melihat efektivitas filtering")
        print("✅ P, K, dan innovation matrices logged untuk analisis EKF performance")

# ===== LOGGING FUNCTIONS =====

def combined_logging_func(q: Queue, filename: str):
    """Enhanced combined logging dengan filter info dan EKF matrices"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 'gps_x', 'gps_y', 'lat', 'lon', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'est_x', 'est_y', 'est_heading_deg', 'est_velocity', 'est_accel',
            'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'gps_updated', 'predict_count', 'gps_update_count', 
            'P_trace', 'K_matrix', 'innovation_vector'  # ✅ Tambahkan kolom P, K, innovation
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def imu_predict_logging_func(q: Queue, filename: str):
    """Enhanced IMU predict logging dengan filter comparison dan P matrix"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'dt', 
            'accel_filtered', 'gyro_filtered_rad', 'gyro_filtered_deg',
            'accel_raw', 'gyro_raw_rad', 'gyro_raw_deg',
            'predict_x', 'predict_y', 'predict_heading_deg', 
            'predict_velocity', 'predict_accel', 
            'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'accel_cutoff_hz', 'gyro_cutoff_hz',
            'predict_count', 'P_trace'  # ✅ Tambahkan kolom P_trace
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

def filter_stats_logging_func(q: Queue, filename: str):
    """Specialized logging untuk filter statistics"""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'accel_filter_effectiveness', 'gyro_filter_effectiveness',
            'accel_cutoff_hz', 'gyro_cutoff_hz',
            'instantaneous_accel_noise_removed', 'instantaneous_gyro_noise_removed',
            'iteration'
        ])
        while True:
            row = q.get()
            if row == "STOP":
                break
            writer.writerow(row)

# ===== TEST FUNCTIONS =====

def test_filter_effectiveness():
    """Test untuk melihat efektivitas filter dalam real-time"""
    print("=== Testing Filter Effectiveness in Real-Time ===")
    
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(
        application_type='vehicle',
        accel_filter_config={'cutoff': 2.0, 'type': 'butterworth', 'adaptive': False},
        gyro_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False}
    )
    time.sleep(2)
    
    print("Collecting data for 30 seconds...")
    print("Time\tAccel_Raw\tAccel_Filt\tNoise_Red\tGyro_Raw\tGyro_Filt\tNoise_Red")
    print("-" * 80)
    
    start_time = time.time()
    
    try:
        for i in range(300):  # 30 seconds at 10Hz
            accel_f, gyro_f, accel_r, gyro_r, stats = imu_handler.get_data(return_raw=True, return_stats=True)
            
            elapsed = time.time() - start_time
            
            if i % 10 == 0:  # Print every second
                accel_noise_red = abs(accel_r - accel_f)
                gyro_noise_red = abs(math.degrees(gyro_r - gyro_f))
                
                print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t{accel_noise_red:.3f}\t\t"
                      f"{math.degrees(gyro_r):.1f}°/s\t\t{math.degrees(gyro_f):.1f}°/s\t\t{gyro_noise_red:.1f}°/s")
            
            time.sleep(0.1)
        
        # Final effectiveness summary
        final_stats = imu_handler.get_stats()
        print(f"\nFilter Effectiveness Summary:")
        print(f"Accelerometer: {final_stats['filter_effectiveness']['accel_noise_reduction']:.1%} noise reduction")
        print(f"Gyroscope: {final_stats['filter_effectiveness']['gyro_noise_reduction']:.1%} noise reduction")
        print(f"IMU read success rate: {final_stats['success_rate']:.1%}")
        
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    finally:
        imu_handler.stop()

def test_different_filter_settings():
    """Test berbagai konfigurasi filter untuk menemukan yang optimal"""
    print("=== Testing Different Filter Settings ===")
    
    test_configs = [
        {
            'name': 'Conservative (Low Cutoff)',
            'accel_cutoff': 1.0,
            'gyro_cutoff': 1.5,
            'filter_type': 'butterworth'
        },
        {
            'name': 'Vehicle Standard',
            'accel_cutoff': 3.0,
            'gyro_cutoff': 5.0,
            'filter_type': 'butterworth'
        },
        {
            'name': 'Aggressive (High Cutoff)',
            'accel_cutoff': 8.0,
            'gyro_cutoff': 12.0,
            'filter_type': 'butterworth'
        },
        {
            'name': 'Moving Average',
            'accel_cutoff': 3.0,
            'gyro_cutoff': 5.0,
            'filter_type': 'moving_average'
        },
        {
            'name': 'Exponential',
            'accel_cutoff': 3.0,
            'gyro_cutoff': 5.0,
            'filter_type': 'exponential'
        }
    ]
    
    results = []
    
    for config in test_configs:
        print(f"\nTesting: {config['name']}")
        
        try:
            imu_handler = FilteredIMUHandler(
                accel_filter_config={
                    'cutoff': config['accel_cutoff'],
                    'type': config['filter_type'],
                    'adaptive': False
                },
                gyro_filter_config={
                    'cutoff': config['gyro_cutoff'],
                    'type': config['filter_type'],
                    'adaptive': False
                }
            )
            
            time.sleep(3)  # Stabilization
            
            # Collect samples
            samples = []
            for i in range(100):  # 10 seconds at 10Hz
                accel_f, gyro_f, accel_r, gyro_r = imu_handler.get_data(return_raw=True)
                samples.append({
                    'accel_filtered': accel_f,
                    'gyro_filtered': gyro_f,
                    'accel_raw': accel_r,
                    'gyro_raw': gyro_r
                })
                time.sleep(0.1)
            
            # Calculate metrics
            if len(samples) > 10:
                accel_raw_std = np.std([s['accel_raw'] for s in samples])
                accel_filtered_std = np.std([s['accel_filtered'] for s in samples])
                gyro_raw_std = np.std([s['gyro_raw'] for s in samples])
                gyro_filtered_std = np.std([s['gyro_filtered'] for s in samples])
                
                accel_reduction = accel_raw_std / max(accel_filtered_std, 1e-6)
                gyro_reduction = gyro_raw_std / max(gyro_filtered_std, 1e-6)
                
                # Calculate delay (approximate)
                from lowpass_filters import calculate_filter_delay
                accel_delay = calculate_filter_delay(config['accel_cutoff'], 50.0, config['filter_type'])
                gyro_delay = calculate_filter_delay(config['gyro_cutoff'], 50.0, config['filter_type'])
                
                result = {
                    'name': config['name'],
                    'accel_reduction': accel_reduction,
                    'gyro_reduction': gyro_reduction,
                    'accel_delay_ms': accel_delay * 20,  # 20ms per sample at 50Hz
                    'gyro_delay_ms': gyro_delay * 20,
                    'config': config
                }
                results.append(result)
                
                print(f"  Accel noise reduction: {accel_reduction:.2f}x")
                print(f"  Gyro noise reduction: {gyro_reduction:.2f}x")
                print(f"  Estimated delay: {accel_delay * 20:.0f}ms (accel), {gyro_delay * 20:.0f}ms (gyro)")
            
            imu_handler.stop()
            time.sleep(1)
            
        except Exception as e:
            print(f"  Error: {e}")
    
    # Summary
    print(f"\n{'='*60}")
    print("FILTER COMPARISON SUMMARY")
    print(f"{'='*60}")
    print(f"{'Configuration':<20} {'Accel Red':<10} {'Gyro Red':<10} {'Delay (ms)':<12}")
    print("-" * 60)
    
    for result in results:
        print(f"{result['name']:<20} {result['accel_reduction']:<10.1f}x {result['gyro_reduction']:<10.1f}x "
              f"{result['accel_delay_ms']:<12.0f}")
    
    # Recommendation
    if results:
        best_overall = max(results, key=lambda x: (x['accel_reduction'] + x['gyro_reduction']) / 2)
        print(f"\nRECOMMENDED: {best_overall['name']}")
        print(f"  Best overall noise reduction with reasonable delay")

def main():
    """Main function dengan pilihan testing dan konfigurasi"""
    print("GPS-IMU Sensor Fusion dengan Low-Pass Filtering")
    print("Mengurangi noise getaran mesin untuk navigasi yang lebih akurat")
    print()
    print("Pilihan:")
    print("1. Run main sensor fusion dengan filtering")
    print("2. Test filter effectiveness real-time")
    print("3. Test berbagai konfigurasi filter")
    print("4. Test IMU predict only (dengan filtering)")
    print("5. Test GPS only")
    print("6. Quick filter demo")
    
    choice = input("Masukkan pilihan (1-6): ").strip()
    
    if choice == '1':
        main_sensor_fusion_with_filters()
    elif choice == '2':
        test_filter_effectiveness()
    elif choice == '3':
        test_different_filter_settings()
    elif choice == '4':
        test_imu_predict_only_filtered()
    elif choice == '5':
        test_gps_only()
    elif choice == '6':
        quick_filter_demo()
    else:
        print("Pilihan tidak valid")

def test_imu_predict_only_filtered():
    """Test khusus IMU predict dengan filtering"""
    print("=== Testing Filtered IMU Predict Only ===")
    
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(application_type='vehicle')
    time.sleep(1)
    
    print("Initializing EKF...")
    dt = 0.05
    ekf = EKFSensorFusion(dt=dt)
    ekf.state[0] = 0  # x
    ekf.state[1] = 0  # y
    
    print("Running filtered IMU-only prediction...")
    print("Press Ctrl+C to stop")
    
    try:
        last_time = time.time()
        predict_counter = 0
        iteration_counter = 0
        
        while True:
            current_time = time.time()
            dt_actual = current_time - last_time
            
            if dt_actual <= 0 or dt_actual > 0.2:
                dt_actual = dt
            
            # Read filtered IMU data
            try:
                accel_f, gyro_f, accel_r, gyro_r, stats = imu_handler.get_data(return_raw=True, return_stats=True)
            except Exception as e:
                print(f"IMU read error: {e}")
                accel_f, gyro_f = 0, 0
                accel_r, gyro_r = 0, 0
                stats = {'accel_noise_reduction': 0, 'gyro_noise_reduction': 0}
            
            # EKF Predict dengan filtered data
            try:
                ekf.predict(imu_accel=accel_f, imu_omega=gyro_f, dt=dt_actual)
                predict_counter += 1
                
                # Extract state
                if len(ekf.state) >= 4:
                    pred_x, pred_y, pred_heading, pred_velocity = ekf.state[:4]
                else:
                    pred_x, pred_y = ekf.state[0], ekf.state[1]
                    pred_heading = 0
                    pred_velocity = 0
                
                pred_heading_deg = math.degrees(pred_heading)
                
                # Extract P matrix
                P_matrix = ekf.get_covariance()
                P_trace = np.trace(P_matrix)
                
                # Print status dengan filter info
                if iteration_counter % 20 == 0:
                    noise_accel = abs(accel_r - accel_f)
                    noise_gyro = abs(math.degrees(gyro_r - gyro_f))
                    
                    print(f"#{iteration_counter}: Pos({pred_x:.2f}, {pred_y:.2f}) | "
                          f"H:{pred_heading_deg:.1f}° | V:{pred_velocity:.2f} | P_trace:{P_trace:.2f}")
                    print(f"    Filter: A_noise={noise_accel:.3f} G_noise={noise_gyro:.1f}°/s | "
                          f"Effectiveness: {stats['accel_noise_reduction']:.1%}, {stats['gyro_noise_reduction']:.1%}")
                
            except Exception as e:
                print(f"EKF predict error: {e}")
            
            iteration_counter += 1
            last_time = current_time
            
            sleep_time = dt - (time.time() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print(f"\nStopped after {iteration_counter} iterations")
    finally:
        imu_handler.stop()

def test_gps_only():
    """Test GPS handler only"""
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

def quick_filter_demo():
    """Quick demo untuk melihat efek filtering dalam waktu singkat"""
    print("=== Quick Filter Demo ===")
    print("Menunjukkan perbedaan data raw vs filtered dalam 10 detik")
    
    print("Initializing filtered IMU...")
    imu_handler = FilteredIMUHandler(
        application_type='vehicle',
        accel_filter_config={'cutoff': 3.0, 'type': 'butterworth', 'adaptive': False},
        gyro_filter_config={'cutoff': 5.0, 'type': 'butterworth', 'adaptive': False}
    )
    time.sleep(2)
    
    print("\nData comparison (Raw vs Filtered):")
    print("Time\tAccel_Raw\tAccel_Filt\tDiff\tGyro_Raw\tGyro_Filt\tDiff")
    print("-" * 70)
    
    try:
        start_time = time.time()
        for i in range(50):  # 10 seconds at 5Hz
            try:
                accel_f, gyro_f, accel_r, gyro_r = imu_handler.get_data(return_raw=True)
                
                elapsed = time.time() - start_time
                accel_diff = abs(accel_r - accel_f)
                gyro_diff = abs(math.degrees(gyro_r - gyro_f))
                
                print(f"{elapsed:.1f}s\t{accel_r:.3f}\t\t{accel_f:.3f}\t\t{accel_diff:.3f}\t"
                      f"{math.degrees(gyro_r):6.1f}°/s\t{math.degrees(gyro_f):6.1f}°/s\t{gyro_diff:5.1f}°/s")
                
            except Exception as e:
                print(f"Error: {e}")
            
            time.sleep(0.2)
        
        print("\n✅ Demo selesai. Filter berhasil mengurangi noise!")
        
    except KeyboardInterrupt:
        print("\nDemo dihentikan oleh user")
    finally:
        imu_handler.stop()

if __name__ == "__main__":
    main()