import smbus2
import time
import math
import logging
import json
import os

log = logging.getLogger("YUVAAN.COMPASS")

class CompassDriver:
    """
    Universal Magnetometer Driver for Drone GPS Modules.
    Supports: QMC5883L (0x0D), HMC5883L (0x1E), IST8310 (0x0E).
    """

    def __init__(self, bus_id=1):
        self.bus_id = bus_id
        self.address = None
        self.chip_type = None
        self.offsets = {"x": 0, "y": 0, "z": 0}
        self.config_path = "config/mag_calibration.json"
        self.connected = False
        
        self._bus_obj = None
        
        self.load_calibration()
        self._find_device()

    def _get_bus(self):
        if self._bus_obj is None:
            try:
                self._bus_obj = smbus2.SMBus(self.bus_id)
            except Exception as e:
                log.error(f"COMPASS: Could not open I2C bus {self.bus_id}: {e}")
        return self._bus_obj

    def _find_device(self):
        bus = self._get_bus()
        if not bus: return
        
        # Addresses: 0x0D (QMC), 0x1E (HMC), 0x0E (IST)
        for addr, name in [(0x0D, "QMC5883L"), (0x1E, "HMC5883L"), (0x0E, "IST8310")]:
            try:
                # Check Chip ID if possible
                who_am_i = bus.read_byte_data(addr, 0x00)
                if (name == "IST8310" and who_am_i == 0x10) or (name != "IST8310"):
                    self.address = addr
                    self.chip_type = name
                    log.info(f"COMPASS: Detected {name} at address {hex(addr)} (ID: {hex(who_am_i)})")
                    self._init_chip(bus)
                    return
            except Exception:
                continue
        log.warning("COMPASS: No magnetometer detected on I2C bus.")

    def _init_chip(self, bus):
        try:
            if self.chip_type == "QMC5883L":
                bus.write_byte_data(self.address, 0x09, 0x1D) 
                bus.write_byte_data(self.address, 0x0B, 0x01) 
            elif self.chip_type == "HMC5883L":
                bus.write_byte_data(self.address, 0x00, 0x70)
                bus.write_byte_data(self.address, 0x01, 0xA0)
                bus.write_byte_data(self.address, 0x02, 0x00)
            elif self.chip_type == "IST8310":
                # Soft Reset
                bus.write_byte_data(self.address, 0x0B, 0x01)
                time.sleep(0.1)
                # Cross-axis calibration (factory compensation)
                bus.write_byte_data(self.address, 0x41, 0x24)
                bus.write_byte_data(self.address, 0x42, 0x21)
                # Average 16 samples per measurement for noise reduction
                bus.write_byte_data(self.address, 0x41, 0x24)
                # Trigger first single measurement (0x01 to CNTL1)
                # IST8310 does NOT have a true continuous mode.
                # Each read must be preceded by a single-measurement trigger.
                bus.write_byte_data(self.address, 0x0A, 0x01)
                time.sleep(0.01)
        except Exception as e:
            log.error(f"COMPASS: Failed to initialize {self.chip_type}: {e}")

    def load_calibration(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r") as f:
                    self.offsets = json.load(f)
                log.info(f"COMPASS: Loaded calibration offsets: {self.offsets}")
            except Exception as e:
                log.error(f"COMPASS: Error loading calibration: {e}")

    def save_calibration(self):
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, "w") as f:
            json.dump(self.offsets, f)
        log.info(f"COMPASS: Saved calibration offsets: {self.offsets}")

    def calibrate(self, duration=30):
        """Perform hard-iron calibration by spinning the rover."""
        if not self.address:
            log.error("COMPASS: Cannot calibrate, no device detected.")
            return

        bus = self._get_bus()
        if not bus: return

        log.info(f"COMPASS: Starting calibration for {duration} seconds. Please rotate rover 360°...")
        
        start_time = time.time()
        min_x, max_x = 32767, -32768
        min_y, max_y = 32767, -32768
        
        last_log_time = start_time
        while (time.time() - start_time) < duration:
            raw = self._read_raw()
            if raw:
                mx, my, mz = raw
                min_x = min(min_x, mx)
                max_x = max(max_x, mx)
                min_y = min(min_y, my)
                max_y = max(max_y, my)
            
            if time.time() - last_log_time > 5:
                remaining = int(duration - (time.time() - start_time))
                log.info(f"COMPASS: Calibration in progress... {remaining}s remaining.")
                last_log_time = time.time()

            time.sleep(0.05)
            
        self.offsets["x"] = (min_x + max_x) / 2
        self.offsets["y"] = (min_y + max_y) / 2
        self.save_calibration()
        log.info(f"COMPASS: Calibration complete. Offsets: {self.offsets}")

    def _read_raw(self):
        if not self.address: return None
        bus = self._get_bus()
        if not bus: return None
        
        try:
            if self.chip_type == "QMC5883L":
                data = bus.read_i2c_block_data(self.address, 0x00, 6)
                x = self._to_int16(data[1], data[0])
                y = self._to_int16(data[3], data[2])
                z = self._to_int16(data[5], data[4])
            elif self.chip_type == "HMC5883L":
                data = bus.read_i2c_block_data(self.address, 0x03, 6)
                x = self._to_int16(data[0], data[1])
                z = self._to_int16(data[2], data[3])
                y = self._to_int16(data[4], data[5])
            elif self.chip_type == "IST8310":
                try:
                    # IST8310 requires a single-measurement trigger before each read.
                    # Write 0x01 to CNTL1 (reg 0x0A) to start a measurement, then
                    # wait for conversion (~6ms at default ODR) before reading.
                    bus.write_byte_data(self.address, 0x0A, 0x01)
                    time.sleep(0.008)  # 8ms to be safe (spec says ~6.4ms)
                    data = bus.read_i2c_block_data(self.address, 0x03, 6)
                    x = self._to_int16(data[1], data[0])
                    y = self._to_int16(data[3], data[2])
                    z = self._to_int16(data[5], data[4])
                    return (x, y, z)
                except Exception:
                    # If read fails, try to re-trigger silently
                    try: bus.write_byte_data(self.address, 0x0A, 0x01)
                    except: pass
                    return None
            else: return None
            return (x, y, z)
        except Exception as e: 
            # On error, try to re-init mode silently
            try: bus.write_byte_data(self.address, 0x0A, 0x01)
            except: pass
            log.error(f"COMPASS: General read error: {e}")
            return None

    def read_heading(self):
        raw = self._read_raw()
        if not raw:
            self.connected = False
            return 0.0
        
        self.connected = True
        x = raw[0] - self.offsets["x"]
        y = raw[1] - self.offsets["y"]
        
        heading = math.atan2(y, x)
        if heading < 0: heading += 2 * math.pi
        return round(math.degrees(heading), 1)

    def _to_int16(self, msb, lsb):
        val = (msb << 8) | lsb
        if val > 32767: val -= 65536
        return val

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    comp = CompassDriver()
    while True:
        print(f"Heading: {comp.read_heading()}°")
        time.sleep(0.5)
