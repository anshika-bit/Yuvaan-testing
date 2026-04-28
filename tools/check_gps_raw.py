import serial
import time
import sys

def parse_nmea_gga(line):
    """
    Parses $GNGGA or $GPGGA for Lat/Long
    Example: $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    """
    parts = line.split(',')
    if len(parts) < 10 or 'GGA' not in parts[0]:
        return None
    
    try:
        # Latitude
        lat_v = float(parts[2])
        lat = (lat_v // 100) + (lat_v % 100) / 60
        if parts[3] == 'S': lat = -lat
        
        # Longitude
        lng_v = float(parts[4])
        lng = (lng_v // 100) + (lng_v % 100) / 60
        if parts[5] == 'W': lng = -lng
        
        return {
            "lat": round(lat, 6),
            "lng": round(lng, 6),
            "sats": int(parts[7]),
            "fix": int(parts[6]) > 0
        }
    except:
        return None

def check_gps(port='/dev/serial0', baud=38400):
    print(f"--- YUVAAN GPS REAL-TIME FIX (38400 Baud) ---")
    print(f"Opening {port}...")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print("Waiting for GPS Lock...")
        
        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if 'GGA' in line:
                data = parse_nmea_gga(line)
                if data:
                    status = "FIX OK" if data['fix'] else "NO FIX"
                    print(f"[{status}] Lat: {data['lat']} | Lng: {data['lng']} | Sats: {data['sats']}")
                else:
                    print(f"Searching... (Raw: {line[:30]}...)")
            elif 'GNTXT' in line:
                print(f"Status: {line}")
                
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"\nERROR: {e}")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/serial0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 38400
    check_gps(port, baud)
