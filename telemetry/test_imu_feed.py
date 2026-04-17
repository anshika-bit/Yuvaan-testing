import requests
import time
import random

# Replace this with your Raspberry Pi's actual IP address or use 127.0.0.1 for local testing
PI_IP = "127.0.0.1" 
URL = f"http://{PI_IP}:5000/api/imu/update"

print("Starting to feed IMU data to the backend...")
print(f"Sending to: {URL}")

try:
    while True:
        # In the future, replace this block with actual I2C sensor reading, e.g.:
        # data = sensor.get_accel_data()
        # pitch, roll, yaw = calculate_angles(data)
        
        # Here we simulate the sensor reading loop for testing:
        pitch = random.uniform(-10.0, 10.0)
        roll = random.uniform(-10.0, 10.0)
        yaw = random.uniform(0.0, 360.0)
        
        payload = {
            "pitch": pitch,
            "roll": roll,
            "yaw": yaw
        }
        
        # Send POST request to update the backend's global state
        response = requests.post(URL, json=payload)
        
        print(f"Sent: {payload} | Response: {response.json()}")
        
        time.sleep(0.5) # Send at 2Hz for testing logic loop
        
except KeyboardInterrupt:
    print("\nStopped feeding IMU data.")
