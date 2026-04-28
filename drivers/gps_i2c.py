"""
YUVAAN GPS Driver — drivers/gps_i2c.py
Hardware: u-blox NEO-M10 or NEO-M8N via I2C (Address 0x42)

Polling-based GPS access. Reads NMEA sentences from u-blox I2C registers.
Address sharing: Can share SDA/SCL pins with MPU9250 (0x68).

Usage:
    from drivers.gps_i2c import get_gps_driver
    gps = get_gps_driver()
    data = gps.read()
"""

import threading
import time
import logging
from smbus2 import SMBus, i2c_msg

log = logging.getLogger("YUVAAN.GPS")

def _parse_nmea_gga(sentence: str) -> dict | None:
    """Parse a GPGGA / GNGGA sentence. Returns dict or None if invalid."""
    try:
        parts = sentence.split(',')
        if len(parts) < 10:
            return None

        lat_raw = parts[2]
        lat_dir = parts[3]
        lng_raw = parts[4]
        lng_dir = parts[5]
        fix_quality = int(parts[6]) if parts[6] else 0
        alt = float(parts[9]) if parts[9] else 0.0

        if not lat_raw or not lng_raw:
            return None

        # Convert DDMM.MMMM to decimal degrees
        lat_deg = int(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        lat = lat_deg + lat_min / 60.0
        if lat_dir == 'S':
            lat = -lat

        lng_deg = int(lng_raw[:3])
        lng_min = float(lng_raw[3:])
        lng = lng_deg + lng_min / 60.0
        if lng_dir == 'W':
            lng = -lng

        return {'lat': round(lat, 6), 'lng': round(lng, 6), 'alt': round(alt, 1), 'fix': fix_quality > 0}

    except Exception:
        return None


class GPSDriverI2C:
    """
    GPS I2C reader for u-blox modules.
    Polls the I2C stream (0xFF) at ~5Hz to retrieve NMEA data.
    """

    def __init__(self, bus: int = 1, address: int = 0x42):
        self.bus_id = bus
        self.address = address
        self.connected = False
        self._latest = {'lat': None, 'lng': None, 'alt': None, 'fix': False}
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        log.info(f"GPS: Initializing I2C (Address: {hex(self.address)})")
        try:
            with SMBus(self.bus_id) as bus:
                # Test connectivity
                try:
                    bus.write_byte(self.address, 0x00)
                    self.connected = True
                except Exception:
                    log.warning(f"GPS: Module not found at {hex(self.address)}")
                    self.connected = False

                buffer = ""
                while self._running:
                    if not self.connected:
                        # Periodically retry to detect if it was plugged in
                        try:
                            bus.write_byte(self.address, 0x00)
                            self.connected = True
                            log.info("GPS: Module reconnected.")
                        except Exception:
                            time.sleep(5)
                            continue

                    try:
                        # Read 32 bytes at a time for efficiency
                        chunk = bus.read_i2c_block_data(self.address, 0xFF, 32)
                        
                        for b in chunk:
                            if b == 255: # No more data available right now
                                break
                            
                            char = chr(b)
                            if char == '\n':
                                if buffer.startswith(('$GPGGA', '$GNGGA')):
                                    parsed = _parse_nmea_gga(buffer.strip())
                                    if parsed:
                                        with self._lock:
                                            self._latest = parsed
                                buffer = ""
                            else:
                                buffer += char
                                
                    except Exception as e:
                        log.debug(f"GPS: Read error: {e}")
                        self.connected = False
                        time.sleep(1)
                        
                    time.sleep(0.1) # Cool down the polling loop (~10Hz)
        except Exception as e:
            log.error(f"GPS: Cannot open I2C bus {self.bus_id} — {e}")

    def read(self) -> dict:
        """Returns latest GPS data: {'lat', 'lng', 'alt', 'fix', 'connected'}"""
        with self._lock:
            data = self._latest.copy()
            data['connected'] = self.connected
            return data

    def stop(self):
        self._running = False


# Singleton accessor
_driver = None

def get_gps_driver() -> GPSDriverI2C:
    global _driver
    if _driver is None:
        _driver = GPSDriverI2C()
    return _driver


if __name__ == "__main__":
    # Test script
    logging.basicConfig(level=logging.INFO)
    print("Starting GPS I2C Test...")
    gps = get_gps_driver()
    try:
        while True:
            data = gps.read()
            print(f"\rGPS: {data}", end="")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
