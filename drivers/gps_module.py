"""
YUVAAN GPS Driver — drivers/gps_module.py
Hardware: u-blox NEO-M8N or compatible NMEA GPS module via UART/Serial

Low-level GPS access. Reads NMEA sentences, parses latitude/longitude/altitude.

Usage:
    from drivers.gps_module import GPSDriver
    gps = GPSDriver(port='/dev/ttyUSB0', baud=9600)
    data = gps.read()
    print(data)  # {'lat': 18.4485, 'lng': 77.4562, 'alt': 430.0, 'fix': True}
"""

import threading
import time
import logging

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
        num_sats = int(parts[7]) if parts[7] else 0
        alt = float(parts[9]) if parts[9] else 0.0

        if not lat_raw or not lng_raw:
            return {'lat': None, 'lng': None, 'alt': 0.0, 'fix': False, 'sats': num_sats}

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

        return {'lat': round(lat, 6), 'lng': round(lng, 6), 'alt': round(alt, 1), 'fix': fix_quality > 0, 'sats': num_sats}

    except Exception:
        return None


class GPSDriver:
    """
    GPS serial reader. Continuously reads NMEA sentences on a background thread.
    Data is accessible via the .read() method at any time.
    """

    def __init__(self, port: str = '/dev/serial0', baud: int = 38400):
        self.port = port
        self.baud = baud
        self.connected = False
        self._latest = {'lat': None, 'lng': None, 'alt': None, 'fix': False, 'sats': 0}
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while self._running:
            try:
                import serial
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.connected = True
                    log.info(f"GPS: Connected on {self.port} at {self.baud} baud")
                    
                    while self._running:
                        try:
                            line = ser.readline().decode('ascii', errors='ignore').strip()
                            if line:
                                log.debug(f"GPS RAW: {line}")
                            
                            if line.startswith(('$GPGGA', '$GNGGA')):
                                log.debug(f"GPS GGA Sentence: {line}")
                                parsed = _parse_nmea_gga(line)
                                if parsed:
                                    with self._lock:
                                        self._latest = parsed
                        except Exception as e:
                            log.warning(f"GPS read error: {e}")
                            self.connected = False
                            break # Reconnect
            except Exception as e:
                if self.connected:
                    log.error(f"GPS: Connection lost on {self.port} — {e}")
                else:
                    log.error(f"GPS: Failed to open {self.port} — {e}")
                self.connected = False
                time.sleep(5) # Wait before retry

    def read(self) -> dict:
        """Returns latest GPS data: {'lat', 'lng', 'alt', 'fix', 'connected'}"""
        with self._lock:
            data = self._latest.copy()
            data['connected'] = self.connected
            return data

    def stop(self):
        self._running = False


# Singleton accessor
_driver: GPSDriver | None = None


def get_gps_driver(port: str = '/dev/serial0') -> GPSDriver:
    global _driver
    if _driver is None:
        _driver = GPSDriver(port=port)
    return _driver
