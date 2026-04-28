import time
import logging
from drivers.compass import CompassDriver

# Configure logging to see what's happening
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s | %(message)s"
)

def get_cardinal(heading):
    """Converts a heading in degrees to a cardinal direction string."""
    directions = [
        ("North", 0), ("North-East", 45), ("East", 90), ("South-East", 135),
        ("South", 180), ("South-West", 225), ("West", 270), ("North-West", 315),
        ("North", 360)
    ]
    # Find the closest direction
    best_dir = "North"
    min_diff = 360
    for name, angle in directions:
        diff = abs(heading - angle)
        if diff < min_diff:
            min_diff = diff
            best_dir = name
    return best_dir

import sys

def test_compass():
    print("=" * 50)
    print("  YUVAAN COMPASS TEST & CALIBRATION")
    print("=" * 50)
    
    try:
        # Initialize driver
        compass = CompassDriver()
        
        if not compass.address:
            print("ERROR: Compass module not detected on I2C bus.")
            return

        # Check if user wants to calibrate
        if "--calibrate" in sys.argv:
            print("\n[CALIBRATION MODE]")
            print("Please rotate the rover 360 degrees slowly for 30 seconds...")
            compass.calibrate(30)
            print("Calibration complete. Saved to config/mag_calibration.json\n")

        print(f"Reading from {compass.chip_type} at {hex(compass.address)}...")
        print("Rotate the rover to see the heading change.")
        print("Press Ctrl+C to stop.")
        print("-" * 50)

        while True:
            heading = compass.read_heading()
            cardinal = get_cardinal(heading)
            raw = compass._read_raw()
            
            if raw:
                # Print heading, cardinal, and raw values clearly
                print(f"\rHeading: {heading:5.1f}° | Dir: {cardinal:10s} | Raw X:{raw[0]:5d} Y:{raw[1]:5d}", end="")
            else:
                print(f"\rHeading: {heading:5.1f}° | Dir: {cardinal:10s} | Raw: Read Error", end="")
                
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")

if __name__ == "__main__":
    test_compass()
