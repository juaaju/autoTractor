"""This program handles the communication over I2C
between a Jetson Nano and a MPU-9250 Gyroscope/Accelerometer/Magnetometer combo.
Extended from MPU6050 driver to include magnetometer functionality.
"""
 
import smbus
import math
import time
from time import sleep
 
class mpu9250:
 
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = smbus.SMBus(8)
 
    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0
 
    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4
    
    # Magnetometer Scale Modifier (resolution is 0.15 μT/LSB)
    MAGNETOMETER_SCALE_MODIFIER = 0.15

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18
 
    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18
 
    # MPU-9250 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C
 
    ACCEL_XOUT0 = 0x3B
    ACCEL_XOUT1 = 0x3C
    ACCEL_YOUT0 = 0x3D
    ACCEL_YOUT1 = 0x3E
    ACCEL_ZOUT0 = 0x3F
    ACCEL_ZOUT1 = 0x40
 
    TEMP_OUT0 = 0x41
    TEMP_OUT1 = 0x42
 
    GYRO_XOUT0 = 0x43
    GYRO_XOUT1 = 0x44
    GYRO_YOUT0 = 0x45
    GYRO_YOUT1 = 0x46
    GYRO_ZOUT0 = 0x47
    GYRO_ZOUT1 = 0x48
 
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    # Magnetometer registers
    AK8963_ADDRESS = 0x0C   # 0x0C is the address of AK8963 magnetometer
    AK8963_WHO_AM_I = 0x00  # Should return 0x48
    AK8963_INFO = 0x01
    AK8963_ST1 = 0x02       # Data ready status bit 0
    AK8963_XOUT_L = 0x03    # Data registers
    AK8963_XOUT_H = 0x04
    AK8963_YOUT_L = 0x05
    AK8963_YOUT_H = 0x06
    AK8963_ZOUT_L = 0x07
    AK8963_ZOUT_H = 0x08
    AK8963_ST2 = 0x09       # Data overflow bit 3 and data read error status bit 2
    AK8963_CNTL = 0x0A      # Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes
    AK8963_ASTC = 0x0C      # Self test control
    AK8963_ASAX = 0x10      # Fuse ROM x-axis sensitivity adjustment value
    AK8963_ASAY = 0x11      # Fuse ROM y-axis sensitivity adjustment value
    AK8963_ASAZ = 0x12      # Fuse ROM z-axis sensitivity adjustment value

    # MPU9250 I2C bypass register
    INT_PIN_CFG = 0x37
    INT_ENABLE = 0x38

    def __init__(self, address):
        self.address = address
        
        # Initialize bias values
        self.accel_bias = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_bias = {'x': 0, 'y': 0, 'z': 0}
        
        # Initialize magnetometer calibration values
        self.mag_offset_x = 0
        self.mag_offset_y = 0
        self.mag_offset_z = 0
        self.mag_scale_x = 1.0
        self.mag_scale_y = 1.0
        self.mag_scale_z = 1.0
        self.mag_sensitivity_x = 1.0
        self.mag_sensitivity_y = 1.0
        self.mag_sensitivity_z = 1.0
 
        # Wake up the MPU-9250 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        sleep(0.1)
        
        # Enable I2C bypass to access magnetometer
        self.enable_magnetometer()
        
        # Set up magnetometer in continuous measurement mode
        self.configure_magnetometer()
        
        # Load factory magnetometer calibration values
        self.load_magnetometer_calibration()

    def calibrate(self):
        """Simple calibration - call this once when sensor is still."""
        print("Calibrating... Keep sensor still for 10 seconds")
        
        accel_sum = {'x': 0, 'y': 0, 'z': 0}
        gyro_sum = {'x': 0, 'y': 0, 'z': 0}
        
        # Take 200 samples
        for i in range(1000):
            accel = self.get_accel_data()
            gyro = self.get_gyro_data()
            
            accel_sum['x'] += accel['x']
            accel_sum['y'] += accel['y']
            accel_sum['z'] += accel['z']
            
            gyro_sum['x'] += gyro['x']
            gyro_sum['y'] += gyro['y']
            gyro_sum['z'] += gyro['z']
            
            sleep(0.01)
        
        # Calculate bias (average)
        self.accel_bias['x'] = accel_sum['x'] / 1000
        self.accel_bias['y'] = accel_sum['y'] / 1000
        self.accel_bias['z'] = (accel_sum['z'] / 1000) - self.GRAVITIY_MS2  # Z should be 9.8 m/s²
        
        self.gyro_bias['x'] = gyro_sum['x'] / 1000
        self.gyro_bias['y'] = gyro_sum['y'] / 1000
        self.gyro_bias['z'] = gyro_sum['z'] / 1000
        
        print("Calibration done!")

    def enable_magnetometer(self):
        """Enable I2C bypass to access the magnetometer."""
        try:
            self.bus.write_byte_data(self.address, self.INT_PIN_CFG, 0x02)  # Enable I2C bypass
            sleep(0.01)
            self.bus.write_byte_data(self.address, self.INT_ENABLE, 0x01)   # Enable raw data ready interrupt
            sleep(0.01)
            print("Magnetometer bypass enabled")
        except Exception as e:
            print(f"Error enabling magnetometer: {e}")

    def configure_magnetometer(self):
        """Configure the AK8963 magnetometer."""
        try:
            # Check if magnetometer is accessible
            who_am_i = self.bus.read_byte_data(self.AK8963_ADDRESS, self.AK8963_WHO_AM_I)
            if who_am_i != 0x48:
                print(f"Warning: Magnetometer not detected (WHO_AM_I: 0x{who_am_i:02X})")
                return False
            
            # Set magnetometer to power down mode
            self.bus.write_byte_data(self.AK8963_ADDRESS, self.AK8963_CNTL, 0x00)
            sleep(0.01)
            
            # Set to continuous measurement mode 2 (100Hz) with 16-bit output
            self.bus.write_byte_data(self.AK8963_ADDRESS, self.AK8963_CNTL, 0x16)
            sleep(0.01)
            
            print("Magnetometer configured successfully")
            return True
        except Exception as e:
            print(f"Error configuring magnetometer: {e}")
            return False

    def load_magnetometer_calibration(self):
        """Load factory calibration values for magnetometer sensitivity."""
        try:
            # Set to Fuse ROM access mode to read sensitivity adjustment values
            self.bus.write_byte_data(self.AK8963_ADDRESS, self.AK8963_CNTL, 0x00)  # Power down
            sleep(0.01)
            self.bus.write_byte_data(self.AK8963_ADDRESS, self.AK8963_CNTL, 0x0F)  # Fuse ROM access
            sleep(0.01)
            
            # Read calibration values
            asax = self.bus.read_byte_data(self.AK8963_ADDRESS, self.AK8963_ASAX)
            asay = self.bus.read_byte_data(self.AK8963_ADDRESS, self.AK8963_ASAY)
            asaz = self.bus.read_byte_data(self.AK8963_ADDRESS, self.AK8963_ASAZ)
            
            # Convert to adjustment factors
            self.mag_sensitivity_x = ((asax - 128) / 256.0) + 1.0
            self.mag_sensitivity_y = ((asay - 128) / 256.0) + 1.0
            self.mag_sensitivity_z = ((asaz - 128) / 256.0) + 1.0
            
            # Return to continuous measurement mode
            self.bus.write_byte_data(self.AK8963_ADDRESS, self.AK8963_CNTL, 0x00)  # Power down
            sleep(0.01)
            self.bus.write_byte_data(self.AK8963_ADDRESS, self.AK8963_CNTL, 0x16)  # Continuous measurement mode 2
            sleep(0.01)
            
            print(f"Magnetometer sensitivity: X={self.mag_sensitivity_x:.3f}, Y={self.mag_sensitivity_y:.3f}, Z={self.mag_sensitivity_z:.3f}")
        except Exception as e:
            print(f"Error loading magnetometer calibration: {e}")
            # Set default values
            self.mag_sensitivity_x = 1.0
            self.mag_sensitivity_y = 1.0
            self.mag_sensitivity_z = 1.0

    # I2C communication methods 
    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
 
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
 
        value = (high << 8) + low
 
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value
            
    def read_magnetometer_word(self, register):
        """Read two registers from magnetometer and combine them."""
        # Read the data from the registers (low byte first for magnetometer)
        low = self.bus.read_byte_data(self.AK8963_ADDRESS, register)
        high = self.bus.read_byte_data(self.AK8963_ADDRESS, register + 1)
        
        value = (high << 8) + low
        
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value
 
    # MPU-9250 Methods
    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-9250.
 
        Returns the temperature in degrees Celcius.
        """
        # Get the raw data
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
 
        # Get the actual temperature using the formula for MPU-9250
        actual_temp = (raw_temp / 333.87) + 21.0
 
        # Return the temperature
        return actual_temp
 
    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
 
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
 
        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)
 
    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.
 
        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        # Get the raw value
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
 
        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1
 
    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer.
 
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        # Read the data from the MPU-9250
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)
 
        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)
 
        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unknown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
 
        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier
 
        if g is True:
            # Apply bias correction in g units
            x = x - (self.accel_bias['x'] / self.GRAVITIY_MS2)
            y = y - (self.accel_bias['y'] / self.GRAVITIY_MS2)
            z = z - (self.accel_bias['z'] / self.GRAVITIY_MS2)
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            # Apply bias correction in m/s² units
            x = x - self.accel_bias['x']
            y = y - self.accel_bias['y']
            z = z - self.accel_bias['z']
            return {'x': x, 'y': y, 'z': z}
 
    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.
 
        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
 
        # Write the new range to the GYRO_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)
 
    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.
 
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        # Get the raw value
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
 
        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1
 
    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.
 
        Returns the read values in a dictionary.
        """
        # Read the raw data from the MPU-9250
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)
 
        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)
 
        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unknown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
 
        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier
        
        # Apply bias correction
        x = x - self.gyro_bias['x']
        y = y - self.gyro_bias['y']
        z = z - self.gyro_bias['z']
 
        return {'x': x, 'y': y, 'z': z}
    
    def get_mag_data(self, apply_calibration=True):
        """Gets and returns the X, Y and Z values from the magnetometer.
        
        If apply_calibration is True, it will apply the calibration values
        Returns the read values in a dictionary in μT.
        """
        try:
            # Check if data is ready
            status = self.bus.read_byte_data(self.AK8963_ADDRESS, self.AK8963_ST1)
            if not (status & 0x01):
                return None  # Data not ready
            
            # Read the raw data from the magnetometer
            x = self.read_magnetometer_word(self.AK8963_XOUT_L)
            y = self.read_magnetometer_word(self.AK8963_YOUT_L)
            z = self.read_magnetometer_word(self.AK8963_ZOUT_L)
            
            # Check for overflow
            status2 = self.bus.read_byte_data(self.AK8963_ADDRESS, self.AK8963_ST2)
            if status2 & 0x08:
                return None  # Overflow detected
                
            # Apply factory sensitivity adjustments
            x = x * self.mag_sensitivity_x
            y = y * self.mag_sensitivity_y
            z = z * self.mag_sensitivity_z
            
            # Convert to microteslas
            x = x * self.MAGNETOMETER_SCALE_MODIFIER
            y = y * self.MAGNETOMETER_SCALE_MODIFIER
            z = z * self.MAGNETOMETER_SCALE_MODIFIER
            
            # Apply calibration (hard iron and soft iron corrections)
            if apply_calibration:
                x = (x - self.mag_offset_x) * self.mag_scale_x
                y = (y - self.mag_offset_y) * self.mag_scale_y
                z = (z - self.mag_offset_z) * self.mag_scale_z
            
            return {'x': x, 'y': y, 'z': z}
        except Exception as e:
            print(f"Error reading magnetometer: {e}")
            return None
    
    def get_compass_heading(self):
        """Calculate heading based on magnetometer data.
        
        Returns heading in degrees from North (0-360).
        """
        mag_data = self.get_mag_data()
        if mag_data is None:
            return None
            
        # Calculate heading
        x = mag_data['x']
        y = mag_data['y']
        
        # Calculate heading in radians and convert to degrees
        heading = math.atan2(y, x)
        
        # Correct for declination (difference between magnetic north and true north)
        # Replace this value with the declination for your location
        declination = 0  # Replace with your local declination in radians
        heading += declination
        
        # Normalize to 0-360 degrees
        heading_deg = math.degrees(heading)
        if heading_deg < 0:
            heading_deg += 360
            
        return heading_deg
 
    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        mag = self.get_mag_data()
        heading = self.get_compass_heading()
 
        return {
            'accel': accel,
            'gyro': gyro,
            'temp': temp,
            'mag': mag,
            'heading': heading
        }
 
if __name__ == "__main__":
    mpu = mpu9250(0x68)
    mpu.calibrate()
    
    while(1):
        try:
            data = mpu.get_all_data()
            
            print("Temperature (C): ", data['temp'])
            
            print("Acceleration x (m/s^2): ", data['accel']['x'])
            print("Acceleration y (m/s^2): ", data['accel']['y'])
            print("Acceleration z (m/s^2): ", data['accel']['z'])
            
            print("Gyroscope x (deg/s): ", data['gyro']['x'])
            print("Gyroscope y (deg/s): ", data['gyro']['y'])
            print("Gyroscope z (deg/s): ", data['gyro']['z'])
            
            if data['mag'] is not None:
                print("Magnetometer x (µT): ", data['mag']['x'])
                print("Magnetometer y (µT): ", data['mag']['y'])
                print("Magnetometer z (µT): ", data['mag']['z'])
                
            if data['heading'] is not None:
                print("Compass Heading: ", data['heading'], "° from North")
                
            print("----------------------------------------")
            sleep(0.2)
            
        except KeyboardInterrupt:
            print("\nProgram terminated by user")
            break
        except Exception as e:
            print(f"Error: {e}")
            sleep(0.5)