import time
import logging
from core.nav_utils import haversine_distance, calculate_bearing, get_bearing_error
from core.bridge import get_bridge

log = logging.getLogger("YUVAAN.NAV")

class NavigationEngine:
    """
    Manages autonomous navigation missions.
    Takes fused sensor data and coordinates movement to waypoints via the Bridge.
    """
    def __init__(self, waypoint_arrival_radius=2.5, bearing_tolerance=10.0):
        self.bridge = get_bridge()
        self.waypoints = [] # List of {'lat', 'lng'}
        self.current_wp_index = 0
        self.active = False
        self.emergency_locked = False
        
        # Configuration
        self.arrival_radius = 1.5 # Increased to account for GPS noise (1.5m is safer)
        self.bearing_tolerance = bearing_tolerance   # Degrees
        self.move_speed_max = 200
        self.move_speed_min = 130
        self.turn_speed = 140 
        self.target_heading_lock = None 
        self.history_path = [] # New: Breadcrumbs for RTL
        self.last_history_pos = None
        self.arrival_latch_threshold = 1.5 
        self.arrival_timer = 0
        self.last_bearing_error = 0
        self.smoothed_brg_error = 0
        self.last_cmd_time = 0
        self.segment_start_pos = None
        self.segment_dist_traveled = 0.0
        self.last_fused_pos = None
        self.expected_segment_dist = 0.0
        self.state = "IDLE" 
        self.status = {
            "active": False,
            "distance_to_wp": 0.0,
            "target_bearing": 0.0,
            "bearing_error": 0.0,
            "current_wp": None,
            "state": "IDLE"
        }

    def set_waypoints(self, wp_list):
        if self.emergency_locked:
            log.error("NAV: Mission Reject. E-STOP IS ACTIVE.")
            return
            
        # Strictly preserve the user's waypoint sequence and actions
        self.waypoints = wp_list
        self.current_wp_index = 0
        self.state = "IDLE"
        log.info(f"NAV: Queued {len(self.waypoints)} exact waypoints.")

    def start(self):
        if self.emergency_locked:
            log.error("NAV: Start Reject. SYSTEM LOCKED.")
            return
        if self.waypoints:
            self.active = True
            self.state = "IDLE"
            log.info("NAV: Mission Started.")
        else:
            log.warning("NAV: Cannot start, no waypoints set.")

    def stop(self, lock=False):
        self.active = False
        self.state = "IDLE"
        # We NO LONGER wipe self.waypoints or self.current_wp_index 
        # so that the user can RESUME the mission after an unlock or manual override.
        if lock:
            self.emergency_locked = True
            log.info("NAV: EMERGENCY STOP. Motors halted. System LOCKED until manual release.")
        else:
            log.info("NAV: Stopped.")
        self.bridge.send_command("STOP")

    def unlock(self):
        self.emergency_locked = False
        log.info("NAV: System Unlocked.")

    def engage_rtl(self, home_pos=None):
        """
        Calculates a safe return path by reversing the history breadcrumbs.
        """
        if not hasattr(self, 'history_path') or len(self.history_path) < 2:
            if home_pos:
                log.warning("NAV: No history breadcrumbs. Heading straight to Home.")
                self.set_waypoints([{'lat': home_pos['lat'], 'lng': home_pos['lng'], 'action': 'end'}])
                self.start()
                return True
            log.error("NAV: Cannot engage RTL. No home reference.")
            return False

        log.info(f"NAV: RTL Engaged. Tracing back {len(self.history_path)} breadcrumbs...")
        
        # Reverse the breadcrumbs to get the path back
        traceback = []
        for p in reversed(self.history_path):
            traceback.append({'lat': p['lat'], 'lng': p['lng'], 'action': 'move'})
        
        # Add final home location if provided
        if home_pos:
            traceback.append({'lat': home_pos['lat'], 'lng': home_pos['lng'], 'action': 'end'})
        else:
            traceback[-1]['action'] = 'end'

        self.set_waypoints(traceback)
        self.start()
        # Clear history so we don't record the RTL path into the return path
        self.history_path = []
        return True

    def update(self, current_pos):
        curr_lat, curr_lng = current_pos.get('lat'), current_pos.get('lng')
        curr_heading = current_pos.get('heading', 0.0)

        # Path Tracking for RTL (Always Record, even in MANUAL Mode)
        if curr_lat is not None and curr_lng is not None:
            if not hasattr(self, 'history_path'):
                self.history_path = []
            
            # Record path if we moved more than 1 meter (prevents noisy history)
            if len(self.history_path) == 0 or haversine_distance(curr_lat, curr_lng, self.history_path[-1]['lat'], self.history_path[-1]['lng']) > 1.0:
                self.history_path.append({'lat': curr_lat, 'lng': curr_lng})
                # Keep last 500 points (~500m of track)
                if len(self.history_path) > 500:
                    self.history_path.pop(0)

        if not self.active or self.emergency_locked:
            self.status["active"] = False
            self.status["state"] = "LOCKED" if self.emergency_locked else "IDLE"
            return self.status

        if self.current_wp_index >= len(self.waypoints):
            log.info("NAV: Mission Complete.")
            self.stop()
            return self.status

        # 1. Get current target
        target = self.waypoints[self.current_wp_index]
        
        if curr_lat is None or curr_lng is None:
            return self.status

        # 2. Calculate errors and track distance moved (Dual-Factor)
        dist = haversine_distance(curr_lat, curr_lng, target['lat'], target['lng'])
        target_brg = calculate_bearing(curr_lat, curr_lng, target['lat'], target['lng'])
        brg_error = get_bearing_error(curr_heading, target_brg)
        
        # Track segment progress via fused odometry
        if self.last_fused_pos:
            moved = haversine_distance(self.last_fused_pos[0], self.last_fused_pos[1], curr_lat, curr_lng)
            self.segment_dist_traveled += moved
        else:
            # Initialize segment on first update or WP switch
            self.segment_start_pos = (curr_lat, curr_lng)
            self.expected_segment_dist = dist
            self.segment_dist_traveled = 0.0
            
        self.last_fused_pos = (curr_lat, curr_lng)
        
        self.status.update({
            "active": True,
            "distance_to_wp": round(dist, 2),
            "target_bearing": round(target_brg, 1),
            "bearing_error": round(brg_error, 1),
            "current_wp": self.current_wp_index,
            "state": self.state
        })

        # 3. Control Loop Logic
        now = time.time()
        
        # Handle Active Actions (like Scanning or Stopping)
        if self.state == "ACTION_WAIT":
            action = target.get('action', 'move').lower()
            
            # Spin Logic (360 degrees)
            if action in ['scan', 'rotate 360']:
                # Continue spinning until timeout
                if now - self.last_cmd_time < 4.5: # Approx 4.5s for 360 spin
                    # Correct Spin Right: Both motors Negative in global PWM space
                    self.bridge.send_command("DRIVE", -160, -160) 
                    return self.status
                else:
                    log.info(f"NAV: Spin/Scan complete for WP {self.current_wp_index}")
            
            # Stop Logic (Pause for 5s then continue)
            elif action == 'stop':
                if now - self.last_cmd_time < 5.0:
                    self.bridge.send_command("STOP")
                    return self.status
                else:
                    log.info(f"NAV: Stop pause complete. Moving to next WP.")

            # Action complete: proceed to next WP
            self.current_wp_index += 1
            self.state = "IDLE"
            self.last_fused_pos = None # Reset segment tracking
            return self.status

        # ARRIVAL CHECK (Dual-Factor: GPS Distance OR Fused Odometry Progress)
        reached_gps = dist < self.arrival_radius
        # Only trust odometry progress if the waypoint is at least 1m away (prevents noise-triggered arrival)
        reached_odo = (self.expected_segment_dist > 1.0) and (self.segment_dist_traveled > (self.expected_segment_dist - 0.5))
        
        if reached_gps or reached_odo:
            # LATCHING: Require 10 consecutive ticks (~0.5s) of "arrival" to filter GPS noise
            if not hasattr(self, 'arrival_ticks'): self.arrival_ticks = 0
            self.arrival_ticks += 1
            
            if self.arrival_ticks < 10:
                # Keep moving forward during the latching window
                pass 
            else:
                log.info(f"NAV: [ARRIVED] WP {self.current_wp_index} (GPS:{reached_gps}, ODO:{reached_odo})")
                self.arrival_ticks = 0
                action = target.get('action', 'move').lower()
                
                if action in ['scan', 'rotate 360', 'stop']:
                    self.state = "ACTION_WAIT"
                    self.last_cmd_time = now
                    self.bridge.send_command("STOP")
                    return self.status
                
                elif action == 'end':
                    log.critical("NAV: Mission SUCCESS. ALL WAYPOINTS REACHED.")
                    self.stop()
                    return self.status
                    
                # Default behavior (start/move): seamless transition
                # Default behavior: Clear locks for next segment
                self.current_wp_index += 1
                self.state = "IDLE"
                self.target_heading_lock = None # Reset lock
                self.last_fused_pos = None 
                return self.status
        else:
            self.arrival_ticks = 0 # Reset if we drift back out
            
        # 1. State Selection (Segmented Map Logic)
        if self.state == "IDLE":
            # EVALUATE THE MAP: Calculate target bearing ONCE at start of segment
            self.target_heading_lock = target_brg 
            
            # Start in MOVING if we are already roughly aligned
            if dist < 4.0 or abs(brg_error) < 45.0:
                self.state = "MOVING"
            else:
                log.info(f"NAV: Alignment Turn Started (Target: {self.target_heading_lock}°, Error: {brg_error:.1f}°)")
                self.state = "TURNING"
            
        elif self.state == "TURNING":
            # USE THE LOCKED HEADING (Ignore real-time bearing jitter)
            # SAFETY: Fallback to target_brg if lock is missing
            t_head = self.target_heading_lock if self.target_heading_lock is not None else target_brg
            locked_error = get_bearing_error(curr_heading, t_head)
            
            if abs(locked_error) < 5.0: 
                log.info(f"NAV: Alignment Complete. Switching to MOVING state.")
                self.state = "MOVING"
            if dist < 2.5:
                self.state = "MOVING"
                
        elif self.state == "MOVING":
            # While moving, we use real-time brg_error for minor steering corrections,
            # but we NEVER go back to axial turning unless the error is catastrophic (>60)
            if dist > 5.0 and abs(brg_error) > 60.0:
                self.state = "TURNING"
                self.target_heading_lock = target_brg
                
        # 2. Execution Logic
        if self.state == "TURNING":
            # Proportional mapping: 180 at max error, 140 at tolerance
            # Add damping: Slow down turn as we get closer to the target heading
            damping = min(1.0, abs(brg_error) / 45.0)
            base_speed = 120 + (damping * 60)
            
            # HARDWARE-MAPPED SIGNS:
            # Right Turn (brg_error > 0) -> L/R both Negative
            # Left Turn (brg_error < 0) -> L/R both Positive
            speed = -int(base_speed) if brg_error > 0 else int(base_speed)
            self.bridge.send_command("DRIVE", speed, speed)
            self.last_cmd_time = now
        else:
            # MOVING - Use differential steering to correct minor errors on the fly
            # 3. ADAPTIVE SPEED: Slow down as we approach the target
            if dist < 4.0:
                # Linear deceleration from move_speed_max to move_speed_min
                target_speed = self.move_speed_min + (dist / 4.0) * (self.move_speed_max - self.move_speed_min)
            else:
                target_speed = self.move_speed_max
 
            steer = int(brg_error * 1.5) 
            
            l_speed = -int(target_speed) - steer
            r_speed = int(target_speed) - steer
            
            # Clamp speeds
            l_speed = max(-255, min(-100, l_speed))
            r_speed = max(100, min(255, r_speed))
            
            self.bridge.send_command("DRIVE", l_speed, r_speed)
            self.last_cmd_time = now

        # 3. Update Status for GCS
        self.status.update({
            "active": self.active,
            "distance_to_wp": round(dist, 1),
            "target_bearing": round(target_brg, 1),
            "bearing_error": round(brg_error, 1),
            "current_wp": self.current_wp_index,
            "state": self.state
        })

        # 4. Record History for RTL (Every 1m of travel)
        if self.last_history_pos is None:
            self.history_path.append({"lat": curr_lat, "lng": curr_lng})
            self.last_history_pos = (curr_lat, curr_lng)
        else:
            h_dist = haversine_distance(self.last_history_pos[0], self.last_history_pos[1], 
                                        curr_lat, curr_lng)
            if h_dist > 1.0:
                self.history_path.append({"lat": curr_lat, "lng": curr_lng})
                self.last_history_pos = (curr_lat, curr_lng)
                if len(self.history_path) > 1000: self.history_path.pop(0)

        return self.status



# Singleton
_nav_engine = None

def get_nav_engine():
    global _nav_engine
    if _nav_engine is None:
        _nav_engine = NavigationEngine()
    return _nav_engine
