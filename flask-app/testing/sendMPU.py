import requests
from time import sleep
from mpu6050 import mpu6050

mpu = mpu6050(0x68)

URL = "http://localhost:5000/update_data"  # Ganti dengan IP Flask server

while True:
    data = {
        "temperature": mpu.get_temp(),
        "accel.x": mpu.get_accel_data()['x'],
    }

    try:
        response = requests.post(URL, json=data)
        print("Response:", response.json())
        print(f"DATA DIKIRIM: {data}")
    except Exception as e:
        print("Error:", e)
    
    sleep(1)
