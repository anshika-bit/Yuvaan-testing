import serial
import requests
import time
import sys

# Configuration
PORT = '/dev/serial0'
BAUD = 9600  # Adjust if your M10 is at 38400
BACKEND_URL = "http://127.0.0.1:5000/api/gps/update"

def parse_nmea_gga(line):
    """
    Very basic GGA parser for Lat/Lng/Alt/Sats
    $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    """
    parts = line.split(',')
    if len(parts) < 10 or parts[0] not in ['$GPGGA', '$GNGGA']:
        return None
    
    try:
        # Latitude
        lat_raw = float(parts[2])
        lat = (lat_raw // 100) + (lat_raw % 100) / 60
        if parts[3] == 'S': lat = -lat
        
        # Longitude
        lng_raw = float(parts[4])
        lng = (lng_raw // 100) + (lng_raw % 100) / 60
        if parts[5] == 'W': lng = -lng
        
        # Other
        fix = int(parts[6]) > 0
        sats = int(parts[7])
        alt = float(parts[9])
        
        return {
            "lat": round(lat, 6),
            "lng": round(lng, 6),
            "alt": round(alt, 1),
            "fix": fix,
            "satellites": sats
        }
    except:
        return None

def main():
    print(f"--- YUVAAN GPS TELEMETRY LINK ---")
    print(f"Port: {PORT} @ {BAUD}")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print("Connected to GPS hardware.")
    except Exception as e:
        print(f"ERROR: Could not open serial port: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                data = parse_nmea_gga(line)
                if data:
                    try:
                        requests.post(BACKEND_URL, json=data, timeout=0.1)
                        print(f"GPS: {data['lat']}, {data['lng']} | Sats: {data['satellites']} | Fix: {data['fix']}", end="\r")
                    except:
                        pass
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Loop error: {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
