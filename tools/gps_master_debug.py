import serial
import time
import os
import smbus2

def scan_i2c_gps():
    print("[1/3] Checking I2C Bus for GPS (0x42)...")
    try:
        bus = smbus2.SMBus(1)
        bus.write_byte(0x42, 0)
        print("  >> FOUND! A GPS module was detected on I2C (Address 0x42).")
        print("  >> ACTION: You should use the I2C driver, not Serial.")
        return True
    except Exception:
        print("  >> Not found on I2C.")
        return False

def debug_serial_port(port, bauds=[9600, 38400, 115200]):
    if not os.path.exists(port):
        return False

    print(f"\n[Checking Serial Port: {port}]")
    for baud in bauds:
        print(f"  Testing {baud} baud...", end="", flush=True)
        try:
            ser = serial.Serial(port, baud, timeout=2)
            # Flush buffers
            ser.reset_input_buffer()
            time.sleep(0.5)
            
            # Read a chunk
            data = ser.read(100)
            ser.close()
            
            if not data:
                print(" No data received.")
                continue
            
            # Analyze data
            try:
                text = data.decode('ascii', errors='ignore')
                if '$GP' in text or '$GN' in text:
                    print(f" SUCCESS! Found NMEA sentences.")
                    print(f"  >> Example: {text.strip().splitlines()[0]}")
                    return True
                else:
                    print(" Received binary/garbage data (Check wiring or baud).")
            except:
                print(" Received non-text data.")
                
        except Exception as e:
            print(f" Error: {e}")
    return False

def main():
    print("="*50)
    print("      YUVAAN GPS MASTER DIAGNOSTIC")
    print("="*50)
    
    # Step 1: I2C Check
    i2c_found = scan_i2c_gps()
    
    # Step 2: Serial Port Check
    print("\n[2/3] Checking Serial Ports...")
    ports_to_check = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0', '/dev/ttyUSB0']
    serial_found = False
    
    for p in ports_to_check:
        if debug_serial_port(p):
            serial_found = True
            break
            
    # Step 3: Final Analysis
    print("\n" + "="*50)
    print("      DIAGNOSTIC SUMMARY")
    print("="*50)
    
    if i2c_found:
        print("RESULT: GPS is on I2C (0x42).")
    elif serial_found:
        print("RESULT: GPS is working on Serial.")
    else:
        print("RESULT: GPS HARDWARE NOT DETECTED.")
        print("\nPossible Issues:")
        print("1. NO POWER: Check if the GPS LED is blinking.")
        print("2. WIRING: Ensure GPS TX -> Pi RX (GPIO 15) and GPS RX -> Pi TX (GPIO 14).")
        print("3. CONFLICT: If you use a Radio for GCS on these pins, move GPS to USB.")
        print("4. CONFIG: Run 'ls -l /dev/serial*' - does it point to ttyS0 or ttyAMA0?")

if __name__ == "__main__":
    main()