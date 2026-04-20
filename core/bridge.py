import serial
import time
import os
import smbus

# --- CONFIGURATION ---
PORT = '/dev/stm32_main'
BAUD = 115200
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47

# Initialize I2C for MPU-9250
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6b, 0) 

def get_gyro_z():
    try:
        high = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG)
        low = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG + 1)
        val = (high << 8) | low
        return val - 65536 if val > 32768 else val
    except:
        return None

def wait_for_stm_complete(ser, keyword="COMPLETE"):
    """Wait for STM32 confirmation before sending next command"""
    print(f"[WAIT] Waiting for STM32 {keyword}...", flush=True)
    start_wait = time.time()
    while (time.time() - start_wait) < 30: # 30s timeout safety
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if keyword in line:
                print(f"[SYNC] Done: {line}", flush=True)
                return True
        time.sleep(0.01)
    print(f"[ERROR] Timeout waiting for {keyword}")
    return False

def execute_90_turn(ser):
    print("\n[TURN] Calibrating IMU for turn...", flush=True)
    bias = 0
    for _ in range(50):
        val = get_gyro_z()
        if val is not None: bias += val
        time.sleep(0.01)
    bias /= 50
    
    print("[TURN] Sending $TURN command...", flush=True)
    ser.write(b"$TURN,90,0*\n") 
    
    heading = 0.0
    last_time = time.time()
    
    while abs(heading) < 88.0:
        curr_time = time.time()
        dt = curr_time - last_time
        last_time = curr_time
        
        raw_z = get_gyro_z()
        if raw_z is not None:
            actual_z = (raw_z - bias) / 131.0
            if abs(actual_z) > 0.15:
                heading += actual_z * dt
            print(f"[IMU] Heading: {heading:7.2f}°", end="\r", flush=True)
        time.sleep(0.02)
    
    ser.write(b"$STOP,0,0*\n")
    print(f"\n[TURN] Completed at {heading:.2f}°", flush=True)
    wait_for_stm_complete(ser, "TURN COMPLETE")

def run_mission():
    print("[SYSTEM] Yuvaan V3.5 Starting...", flush=True)
    while True:
        if not os.path.exists(PORT):
            time.sleep(1)
            continue
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.1)
            print("[SUCCESS] Link Active.", flush=True)
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "MODE: AUT" in line:
                    print("\n[MISSION] SEQUENCE START", flush=True)
                    
                    # 1. First Leg
                    ser.write(b"$MOVE,2.5,0*\n")
                    if not wait_for_stm_complete(ser, "MOVE COMPLETE"): continue
                    
                    time.sleep(1.5)
                    
                    # 2. The 90 Degree Turn
                    execute_90_turn(ser)
                    
                    time.sleep(1.5)
                    
                    # 3. Second Leg
                    ser.write(b"$MOVE,2.5,0*\n")
                    wait_for_stm_complete(ser, "MOVE COMPLETE")
                    
                    print("[MISSION] FINISHED.", flush=True)
                    time.sleep(10) # Cooldown
                time.sleep(0.1)
        except Exception as e:
            print(f"[RECONNECT] {e}", flush=True)
            time.sleep(2)

if __name__ == "__main__":
    run_mission()