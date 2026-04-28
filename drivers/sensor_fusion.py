import time
import math
import logging

log = logging.getLogger("YUVAAN.FUSION")

class SensorFusion:
    """
    Sensor Fusion Engine for YUVAAN Rover.
    Combines GPS (absolute position), IMU (relative orientation), and Encoders (odometry).
    """
    def __init__(self):
        # State
        self.fused_lat = None
        self.fused_lng = None
        self.fused_heading = 0.0
        self.velocity = 0.0
        
        # Odometry State
        self.last_l_enc = 0
        self.last_r_enc = 0
        self.TICKS_PER_METER = 1355.0
        
        # PERSISTENCE: Load last known position
        self.persistence_file = "logs/last_pos.json"
        self._load_persistence()
        
        # Filtering coefficients
        self.gps_weight = 0.8 
        self.gyro_weight = 0.98
        
        self.last_update = time.time()
        
        # Load base origin from environment (for indoor testing without GPS)
        import os
        self.base_lat = float(os.environ.get("YUVAAN_BASE_LAT", "18.4485")) # Default to a real location if not set
        self.base_lng = float(os.environ.get("YUVAAN_BASE_LNG", "77.4562"))

    def update(self, gps_data, imu_data, compass_heading, bridge_data=None):
        """
        gps_data: {'lat', 'lng', 'fix', ...}
        imu_data: {'yaw_rate', ...}
        compass_heading: float
        bridge_data: {'encoders': {'left', 'right'}, ...}
        """
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        # 1. SPEED CALCULATION & DEAD RECKONING (Odometry)
        dist_moved = 0.0
        if bridge_data and bridge_data.get('connected'):
            curr_l = bridge_data['encoders'].get('left', 0)
            curr_r = bridge_data['encoders'].get('right', 0)
            
            # Distance moved since last tick (meters)
            d_l = (curr_l - self.last_l_enc) / self.TICKS_PER_METER
            d_r = (curr_r - self.last_r_enc) / self.TICKS_PER_METER
            
            # SIGNED ODOMETRY: 
            # In our wiring (main.py): Forward is -L and +R
            # So dist_moved = (-d_l + d_r) / 2.0
            # During a SPIN (+L, +R), d_l and d_r are positive, so dist_moved cancels out to ~0.
            dist_moved = (-d_l + d_r) / 2.0
            
            # Filter out giant jumps (encoder glitches)
            if abs(d_l) > 5.0 or abs(d_r) > 5.0:
                dist_moved = 0.0
            
            inst_velocity = dist_moved / dt if dt > 0 else 0
            self.velocity = (0.8 * self.velocity) + (0.2 * abs(inst_velocity))
            
            self.last_l_enc = curr_l
            self.last_r_enc = curr_r
            
            # Dead Reckoning (Relative update)
            if self.fused_lat is not None and abs(dist_moved) > 0.001:
                # Approximate 1 degree lat = 111,111 meters
                rad = math.radians(self.fused_heading)
                delta_lat = (dist_moved * math.cos(rad)) / 111111.0
                delta_lng = (dist_moved * math.sin(rad)) / (111111.0 * math.cos(math.radians(self.fused_lat)))
                self.fused_lat += delta_lat
                self.fused_lng += delta_lng

        # 2. HEADING FUSION
        if imu_data.get('connected', True):
            gyro_z = imu_data.get('yaw_rate', 0)
            # INTEGRATION FIX: 
            # Compass heading increases Clockwise (N=0, E=90).
            # Gyro Z (MPU9250) is Positive Counter-Clockwise.
            # We must SUBTRACT gyro_z to align with the compass direction.
            predicted_heading = self.fused_heading - (gyro_z * dt)
            
            # Trust the Gyro 99% for short-term changes to ignore magnetic interference from motors
            self.fused_heading = (0.99 * predicted_heading) + (0.01 * compass_heading)
        else:
            self.fused_heading = compass_heading
        
        self.fused_heading %= 360

        # 3. POSITION RE-ANCHORING (GPS Correction)
        if gps_data.get('connected', True) and gps_data.get('fix'):
            if self.fused_lat is None:
                self.fused_lat = gps_data['lat']
                self.fused_lng = gps_data['lng']
                log.info(f"FUSION: Re-anchored to GPS Base: {self.fused_lat}, {self.fused_lng}")
            else:
                # ZERO-VELOCITY LOCK: If we aren't physically moving, ignore GPS jitter.
                # This prevents the "drunk walk" when the rover is idle.
                is_actually_moving = self.velocity > 0.15 or abs(dist_moved) > 0.01
                
                if is_actually_moving:
                    # LOW PASS FILTER: Slowly pull towards GPS when moving
                    weight = 0.2 if gps_data.get('sats', 0) > 6 else 0.05
                    self.fused_lat = (weight * gps_data['lat']) + ((1 - weight) * self.fused_lat)
                    self.fused_lng = (weight * gps_data['lng']) + ((1 - weight) * self.fused_lng)
                else:
                    # Stationary: Average the GPS samples to filter noise and lock position
                    avg_weight = 0.02 # Very slow pull to average out the jitter
                    self.fused_lat = (avg_weight * gps_data['lat']) + ((1 - avg_weight) * self.fused_lat)
                    self.fused_lng = (avg_weight * gps_data['lng']) + ((1 - avg_weight) * self.fused_lng)

        elif self.fused_lat is None:
            # NO GPS FIX AND NO FUSED POSITION YET: Use Base Station Fallback
            self.fused_lat = self.base_lat
            self.fused_lng = self.base_lng
            log.warning(f"FUSION: No GPS. Using Dead-Reckoning Base: {self.fused_lat}, {self.fused_lng}")
        
        # 4. Save Persistence (Once every few seconds)
        if now % 2 < 0.1: # Approx every 2s
            self._save_persistence()

        return {
            "lat": self.fused_lat,
            "lng": self.fused_lng,
            "heading": round(self.fused_heading, 2),
            "velocity": round(self.velocity, 2),
            "fix": gps_data.get('fix', False),
            "sats": gps_data.get('sats', 0),
            "pitch": imu_data.get('pitch', 0),
            "roll": imu_data.get('roll', 0),
            "gps_connected": gps_data.get('connected', False),
            "imu_connected": imu_data.get('connected', False),
            "mag_connected": gps_data.get('mag_connected', False)
        }


    def _load_persistence(self):
        import json, os
        if os.path.exists(self.persistence_file):
            try:
                with open(self.persistence_file, 'r') as f:
                    data = json.load(f)
                    self.fused_lat = data.get('lat')
                    self.fused_lng = data.get('lng')
                    log.info(f"FUSION: Loaded persisted position: {self.fused_lat}, {self.fused_lng}")
            except: pass

    def _save_persistence(self):
        import json
        if self.fused_lat is not None:
            try:
                with open(self.persistence_file, 'w') as f:
                    json.dump({'lat': self.fused_lat, 'lng': self.fused_lng}, f)
            except: pass

def get_fusion_engine():
    return SensorFusion()
