import serial
import time
import os
import smbus

# --- SETTINGS ---
PORT = '/dev/stm32_main'
BAUD = 115200
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47

# Initialize I2C for MPU
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6b, 0) # Wake up MPU

def get_gyro_z():
    try:
        high = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG)
        low = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG + 1)
        val = (high << 8) | low
        return val - 65536 if val > 32768 else val
    except:
        return None

def execute_90_turn(ser):
    print("[TURN] Starting 90-degree autonomous turn...", flush=True)
    
    # 1. Calibration for Bias (Finding the drift while standing still)
    bias = 0
    for _ in range(50):
        val = get_gyro_z()
        if val is not None: bias += val
        time.sleep(0.01)
    bias /= 50
    
    # 2. Start Spinning (Send TURN command to STM32)
    # We use a value like 90 just to tell the STM32 to enter its turn loop
    ser.write(b"$TURN,90,0*\n") 
    
    heading = 0.0
    last_time = time.time()
    
    # 3. Monitor IMU until 90 degrees is reached
    while abs(heading) < 88.0: # Stop slightly early to account for momentum
        curr_time = time.time()
        dt = curr_time - last_time
        last_time = curr_time
        
        raw_z = get_gyro_z()
        if raw_z is not None:
            actual_z = (raw_z - bias) / 131.0
            if abs(actual_z) > 0.15: # Noise gate
                heading += actual_z * dt
            print(f"[IMU] Current Heading: {heading:.2f}°", end="\r", flush=True)
        
        time.sleep(0.02)
    
    # 4. Stop the Rover
    ser.write(b"$STOP,0,0*\n")
    print(f"\n[TURN] Completed. Final Heading: {heading:.2f}°", flush=True)

def run_mission():
    print("[SYSTEM] Yuvaan Brain Booting...", flush=True)
    
    while True:
        if not os.path.exists(PORT):
            print(f"[WAIT] {PORT} not found. Check USB.", flush=True)
            time.sleep(2)
            continue

        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.1)
            print(f"[SUCCESS] Connected to Body at {PORT}", flush=True)
            
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[INCOMING] {line}", flush=True)

                    if "MODE: AUT" in line:
                        print("\n[!!!] AUTO MISSION STARTING!", flush=True)
                        
                        # STEP 1: Move 2.5m Forward
                        ser.write(b"$MOVE,2.5,0*\n")
                        print("[CMD] Moving 2.5m...", flush=True)
                        time.sleep(10) # Adjust based on your rover speed
                        
                        # STEP 2: Precise 90 Degree Turn
                        execute_90_turn(ser)
                        time.sleep(2) # Stability pause
                        
                        # STEP 3: Move 2.5m Forward again
                        ser.write(b"$MOVE,2.5,0*\n")
                        print("[CMD] Moving final 2.5m...", flush=True)
                        time.sleep(10)
                        
                        # STEP 4: Mission Complete
                        ser.write(b"$STOP,0,0*\n")
                        print("[MISSION] 2.5m -> 90 Turn -> 2.5m COMPLETE.", flush=True)
                        
                time.sleep(0.1)

        except Exception as e:
            print(f"[RECONNECTING] Lost link: {e}", flush=True)
            time.sleep(2)

if __name__ == "__main__":
    run_mission()