import smbus2
import sys

def scan_i2c(bus_id=1):
    print(f"Scanning I2C bus {bus_id}...")
    bus = smbus2.SMBus(bus_id)
    found = []
    for address in range(0x03, 0x77):
        try:
            bus.write_byte(address, 0)
            print(f"Found device at: {hex(address)}")
            found.append(hex(address))
        except Exception:
            pass
    
    if not found:
        print("No devices found.")
    else:
        print(f"Scan complete. Found: {', '.join(found)}")
    
    # Check known addresses
    if '0x68' in found: print("-> 0x68: MPU9250 (IMU) detected.")
    if '0xe' in found:  print("-> 0x0e: IST8310 (Compass) detected.")
    if '0x42' in found: print("-> 0x42: u-blox GPS (I2C) detected.")

if __name__ == "__main__":
    scan_i2c()
