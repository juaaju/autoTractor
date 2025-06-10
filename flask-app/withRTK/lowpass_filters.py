"""
Low Pass Filter implementation untuk mengurangi noise getaran mesin
File: lowpass_filters.py
"""

import numpy as np
import math
from collections import deque

class LowPassFilter:
    """
    Low-pass filter untuk mengurangi noise getaran mesin
    Menggunakan kombinasi Moving Average dan Exponential Moving Average
    """
    def __init__(self, cutoff_freq=2.0, sampling_freq=50.0, filter_type='butterworth'):
        """
        Args:
            cutoff_freq: Frekuensi cutoff dalam Hz (default 2Hz untuk menghilangkan getaran)
            sampling_freq: Frekuensi sampling sensor dalam Hz
            filter_type: 'butterworth', 'exponential', atau 'moving_average'
        """
        self.cutoff_freq = cutoff_freq
        self.sampling_freq = sampling_freq
        self.filter_type = filter_type
        
        # Initialize filter parameters
        if filter_type == 'butterworth':
            self._init_butterworth()
        elif filter_type == 'exponential':
            self._init_exponential()
        elif filter_type == 'moving_average':
            self._init_moving_average()
        
        # State variables
        self.initialized = False
        self.prev_input = [0.0, 0.0]  # x[n-1], x[n-2]
        self.prev_output = [0.0, 0.0]  # y[n-1], y[n-2]
        self.buffer = deque(maxlen=self.window_size if filter_type == 'moving_average' else 2)
    
    def _init_butterworth(self):
        """Initialize 2nd order Butterworth low-pass filter"""
        # Calculate filter coefficients
        nyquist = self.sampling_freq / 2.0
        normalized_cutoff = self.cutoff_freq / nyquist
        
        # Clamp normalized cutoff to prevent instability
        normalized_cutoff = min(normalized_cutoff, 0.99)
        
        # Butterworth filter design
        omega = math.tan(math.pi * normalized_cutoff)
        k1 = math.sqrt(2) * omega
        k2 = omega * omega
        a0 = k2 + k1 + 1
        
        # Filter coefficients
        self.b0 = k2 / a0  # Current input
        self.b1 = 2 * k2 / a0  # Previous input
        self.b2 = k2 / a0  # Previous previous input
        
        self.a1 = (2 * (k2 - 1)) / a0  # Previous output
        self.a2 = (k2 - k1 + 1) / a0  # Previous previous output
    
    def _init_exponential(self):
        """Initialize exponential moving average filter"""
        # Calculate smoothing factor
        dt = 1.0 / self.sampling_freq
        rc = 1.0 / (2 * math.pi * self.cutoff_freq)
        self.alpha = dt / (rc + dt)
    
    def _init_moving_average(self):
        """Initialize moving average filter"""
        # Window size based on cutoff frequency
        self.window_size = max(3, int(self.sampling_freq / (2 * self.cutoff_freq)))
    
    def filter(self, input_value):
        """
        Apply low-pass filter to input value
        
        Args:
            input_value: Raw sensor value
            
        Returns:
            Filtered value
        """
        if not self.initialized:
            self.initialized = True
            self.prev_input = [input_value, input_value]
            self.prev_output = [input_value, input_value]
            if self.filter_type == 'moving_average':
                for _ in range(self.window_size):
                    self.buffer.append(input_value)
            return input_value
        
        if self.filter_type == 'butterworth':
            return self._butterworth_filter(input_value)
        elif self.filter_type == 'exponential':
            return self._exponential_filter(input_value)
        elif self.filter_type == 'moving_average':
            return self._moving_average_filter(input_value)
    
    def _butterworth_filter(self, x):
        """2nd order Butterworth filter implementation"""
        # y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        output = (self.b0 * x + 
                 self.b1 * self.prev_input[0] + 
                 self.b2 * self.prev_input[1] - 
                 self.a1 * self.prev_output[0] - 
                 self.a2 * self.prev_output[1])
        
        # Update history
        self.prev_input[1] = self.prev_input[0]
        self.prev_input[0] = x
        self.prev_output[1] = self.prev_output[0]
        self.prev_output[0] = output
        
        return output
    
    def _exponential_filter(self, x):
        """Exponential moving average filter"""
        if not hasattr(self, 'ema_output'):
            self.ema_output = x
        
        self.ema_output = self.alpha * x + (1 - self.alpha) * self.ema_output
        return self.ema_output
    
    def _moving_average_filter(self, x):
        """Moving average filter"""
        self.buffer.append(x)
        return sum(self.buffer) / len(self.buffer)
    
    def reset(self):
        """Reset filter state"""
        self.initialized = False
        self.prev_input = [0.0, 0.0]
        self.prev_output = [0.0, 0.0]
        self.buffer.clear()
        if hasattr(self, 'ema_output'):
            delattr(self, 'ema_output')

class AdaptiveLowPassFilter:
    """
    Adaptive low-pass filter yang menyesuaikan cutoff frequency
    berdasarkan tingkat noise yang terdeteksi
    """
    def __init__(self, base_cutoff=2.0, sampling_freq=50.0, adaptation_rate=0.1):
        self.base_cutoff = base_cutoff
        self.sampling_freq = sampling_freq
        self.adaptation_rate = adaptation_rate
        
        self.current_cutoff = base_cutoff
        self.filter = LowPassFilter(base_cutoff, sampling_freq, 'butterworth')
        
        # Noise detection
        self.noise_buffer = deque(maxlen=20)
        self.prev_value = None
        
    def filter_adaptive(self, input_value):
        """Apply adaptive filtering"""
        # Detect noise level
        if self.prev_value is not None:
            noise_level = abs(input_value - self.prev_value)
            self.noise_buffer.append(noise_level)
            
            # Calculate average noise
            if len(self.noise_buffer) >= 5:
                avg_noise = sum(self.noise_buffer) / len(self.noise_buffer)
                
                # Adapt cutoff frequency based on noise level
                if avg_noise > 0.5:  # High noise - lower cutoff (more filtering)
                    target_cutoff = max(0.5, self.base_cutoff * 0.5)
                elif avg_noise < 0.1:  # Low noise - higher cutoff (less filtering)
                    target_cutoff = min(5.0, self.base_cutoff * 2.0)
                else:
                    target_cutoff = self.base_cutoff
                
                # Smooth cutoff adjustment
                self.current_cutoff += self.adaptation_rate * (target_cutoff - self.current_cutoff)
                
                # Reinitialize filter if cutoff changed significantly
                if abs(self.filter.cutoff_freq - self.current_cutoff) > 0.5:
                    self.filter = LowPassFilter(self.current_cutoff, self.sampling_freq, 'butterworth')
        
        self.prev_value = input_value
        return self.filter.filter(input_value)
    
    def get_current_cutoff(self):
        """Get current adaptive cutoff frequency"""
        return self.current_cutoff
    
    def reset(self):
        """Reset adaptive filter state"""
        self.filter.reset()
        self.noise_buffer.clear()
        self.prev_value = None
        self.current_cutoff = self.base_cutoff

class MultiChannelFilter:
    """
    Multi-channel filter untuk filtering beberapa sensor sekaligus
    dengan korelasi antar channel
    """
    def __init__(self, num_channels=2, cutoff_freq=2.0, sampling_freq=50.0):
        self.num_channels = num_channels
        self.filters = []
        
        # Create individual filters for each channel
        for i in range(num_channels):
            self.filters.append(LowPassFilter(cutoff_freq, sampling_freq, 'butterworth'))
        
        # Cross-correlation detection
        self.channel_history = [deque(maxlen=10) for _ in range(num_channels)]
        
    def filter_multi(self, input_values):
        """
        Filter multiple channels simultaneously
        
        Args:
            input_values: List/array of values for each channel
            
        Returns:
            List of filtered values
        """
        if len(input_values) != self.num_channels:
            raise ValueError(f"Expected {self.num_channels} inputs, got {len(input_values)}")
        
        filtered_values = []
        
        for i, value in enumerate(input_values):
            # Apply individual filter
            filtered = self.filters[i].filter(value)
            filtered_values.append(filtered)
            
            # Store in history for correlation analysis
            self.channel_history[i].append(filtered)
        
        # Optional: Apply cross-channel correlation correction
        # (Implement if needed for your specific use case)
        
        return filtered_values
    
    def get_channel_stats(self):
        """Get statistics for each channel"""
        stats = []
        for i in range(self.num_channels):
            if len(self.channel_history[i]) > 1:
                channel_data = list(self.channel_history[i])
                stats.append({
                    'mean': np.mean(channel_data),
                    'std': np.std(channel_data),
                    'range': max(channel_data) - min(channel_data)
                })
            else:
                stats.append({'mean': 0, 'std': 0, 'range': 0})
        return stats

# Utility functions
def calculate_filter_delay(cutoff_freq, sampling_freq, filter_type='butterworth'):
    """
    Calculate approximate filter delay in samples
    
    Args:
        cutoff_freq: Cutoff frequency in Hz
        sampling_freq: Sampling frequency in Hz
        filter_type: Type of filter
        
    Returns:
        Delay in samples
    """
    if filter_type == 'butterworth':
        # Approximate delay for 2nd order Butterworth
        return int(sampling_freq / (2 * cutoff_freq))
    elif filter_type == 'moving_average':
        window_size = max(3, int(sampling_freq / (2 * cutoff_freq)))
        return window_size // 2
    elif filter_type == 'exponential':
        # Time constant in samples
        return int(sampling_freq / (2 * math.pi * cutoff_freq))
    else:
        return 1

def design_filter_for_application(application_type='vehicle'):
    """
    Design filter parameters for specific applications
    
    Args:
        application_type: 'vehicle', 'drone', 'robot', etc.
        
    Returns:
        Dictionary with recommended filter settings
    """
    if application_type == 'vehicle':
        return {
            'accel_cutoff': 3.0,  # Hz - allow vehicle dynamics, filter engine vibration
            'gyro_cutoff': 5.0,   # Hz - allow turning dynamics
            'filter_type': 'butterworth',
            'adaptive': False
        }
    elif application_type == 'drone':
        return {
            'accel_cutoff': 10.0,  # Higher for fast dynamics
            'gyro_cutoff': 15.0,
            'filter_type': 'butterworth',
            'adaptive': True
        }
    elif application_type == 'walking_robot':
        return {
            'accel_cutoff': 2.0,   # Lower for smooth walking
            'gyro_cutoff': 3.0,
            'filter_type': 'moving_average',
            'adaptive': False
        }
    else:
        # Default conservative settings
        return {
            'accel_cutoff': 2.0,
            'gyro_cutoff': 2.0,
            'filter_type': 'butterworth',
            'adaptive': False
        }

if __name__ == "__main__":
    # Test filter functionality
    print("Testing Low Pass Filters...")
    
    # Create test signal with noise
    import numpy as np
    t = np.linspace(0, 1, 100)
    clean_signal = np.sin(2 * np.pi * 2 * t)  # 2Hz signal
    noise = 0.3 * np.random.normal(0, 1, len(t))
    noisy_signal = clean_signal + noise
    
    # Test different filters
    filters = {
        'Butterworth': LowPassFilter(3.0, 100.0, 'butterworth'),
        'Exponential': LowPassFilter(3.0, 100.0, 'exponential'),
        'Moving Average': LowPassFilter(3.0, 100.0, 'moving_average')
    }
    
    print("Filter performance comparison:")
    for name, filt in filters.items():
        filtered = [filt.filter(val) for val in noisy_signal]
        mse = np.mean((np.array(filtered) - clean_signal)**2)
        print(f"{name:15s}: MSE = {mse:.4f}")
    
    print("✓ Filter tests completed")