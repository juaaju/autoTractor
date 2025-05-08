from dfrobotimu import IMU_WT61PCTTL

mpu = IMU_WT61PCTTL(port='/dev/ttyTHS0', baudrate=9600)

while(1):
	data = mpu.get_all_data()
	print(f"x {data['accel']['x']}, z {data['gyro']['z']}", end='\r')