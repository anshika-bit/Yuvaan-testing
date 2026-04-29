"""
YUVAAN Navigation Engine — core/navigation.py

Autonomous waypoint navigation with:
  - PID steering controller
  - Pure Pursuit path tracking
  - Adaptive turn speed
  - Terrain-aware speed control (pitch)
  - Stuck detection + recovery
  - Human detection pause
  - Geofence enforcement
  - Sensor degradation fallback
"""

import time
import math
import logging
from core.nav_utils import haversine_distance, calculate_bearing, get_bearing_error
from core.bridge import get_bridge
from core import nav_config

log = logging.getLogger("YUVAAN.NAV")


class PIDController:
    """Discrete PID controller for bearing error → steering correction."""

    def __init__(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def compute(self, error, dt):
        cfg = nav_config.get_config()
        kp = cfg["pid_kp"]
        ki = cfg["pid_ki"]
        kd = cfg["pid_kd"]
        i_max = cfg["pid_integral_max"]

        # P
        p = kp * error

        # I with anti-windup
        self.integral += error * dt
        self.integral = max(-i_max, min(i_max, self.integral))
        i = ki * self.integral

        # D (on error derivative)
        d = 0.0
        if dt > 0:
            d = kd * (error - self.last_error) / dt
        self.last_error = error

        output = p + i + d
        return max(-255, min(255, output))


class NavigationEngine:
    """Manages autonomous navigation missions with multi-sensor fusion."""

    def __init__(self):
        self.bridge = get_bridge()
        self.pid = PIDController()
        self.waypoints = []
        self.current_wp_index = 0
        self.active = False
        self.emergency_locked = False

        # State machine
        self.state = "IDLE"
        self.target_heading_lock = None

        # RTL history
        self.history_path = []
        self.last_history_pos = None

        # Arrival detection
        self.arrival_ticks = 0
        self.segment_start_pos = None
        self.segment_dist_traveled = 0.0
        self.last_fused_pos = None
        self.expected_segment_dist = 0.0

        # Timing
        self.last_cmd_time = 0

        # Stuck detection
        self.stuck_timer = 0.0
        self.stuck_count = 0
        self.last_velocity = 0.0
        self.last_update_time = 0.0

        # Human pause
        self.human_pause_active = False
        self.last_human_detection_time = 0.0

        # Geofence
        self.home_position = None

        # Status output
        self.status = {
            "active": False, "state": "IDLE",
            "distance_to_wp": 0.0, "target_bearing": 0.0,
            "bearing_error": 0.0, "cross_track_error": 0.0,
            "current_wp": None, "total_wps": 0,
            "velocity": 0.0, "pitch": 0.0,
            "eta_seconds": 0, "confidence": 1.0,
            "stuck_count": 0, "mission_progress": 0.0,
            "mode": "DIRECT", "pid_output": 0.0,
        }

        # Throttled diagnostics for live field debugging
        self._last_block_reason = None
        self._last_block_log_at = 0.0
        self._last_progress_log_at = 0.0
        self._last_cmd_signature = None
        self._last_cmd_log_at = 0.0

    def _log_blocked(self, reason, now, detail=""):
        if self._last_block_reason != reason or (now - self._last_block_log_at) > 2.0:
            suffix = f" | {detail}" if detail else ""
            log.warning(f"NAV_DEBUG: blocked={reason}{suffix}")
            self._last_block_reason = reason
            self._last_block_log_at = now

    def _log_progress(self, now, *, dist, brg_error, velocity, curr_lat, curr_lng):
        if now - self._last_progress_log_at >= 1.0:
            log.info(
                f"NAV_DEBUG: state={self.state} wp={self.current_wp_index + 1}/{len(self.waypoints)} "
                f"dist={dist:.2f}m err={brg_error:.1f}deg vel={velocity:.2f} "
                f"pos={curr_lat:.6f},{curr_lng:.6f}"
            )
            self._last_progress_log_at = now
            self._last_block_reason = None

    def _log_cmd(self, now, kind, left, right, detail=""):
        signature = (kind, int(left), int(right), detail)
        if self._last_cmd_signature != signature or (now - self._last_cmd_log_at) >= 1.0:
            suffix = f" | {detail}" if detail else ""
            log.info(f"NAV_CMD: {kind} L={int(left)} R={int(right)}{suffix}")
            self._last_cmd_signature = signature
            self._last_cmd_log_at = now

    # ── Waypoint Management ──────────────────────────────────────────
    def set_waypoints(self, wp_list):
        if self.emergency_locked:
            log.error("NAV: Mission Reject. E-STOP IS ACTIVE.")
            return
        self.waypoints = wp_list
        self.current_wp_index = 0
        self.state = "IDLE"
        self.pid.reset()
        self.stuck_count = 0
        log.info(f"NAV: Queued {len(self.waypoints)} waypoints.")

    def start(self):
        if self.emergency_locked:
            log.error("NAV: Start Reject. SYSTEM LOCKED.")
            return
        if self.waypoints:
            self.active = True
            self.state = "IDLE"
            self.pid.reset()
            self.stuck_timer = 0.0
            self.stuck_count = 0
            log.info("NAV: Mission Started.")
        else:
            log.warning("NAV: Cannot start, no waypoints set.")

    def stop(self, lock=False):
        self.active = False
        self.state = "IDLE"
        self.pid.reset()
        if lock:
            self.emergency_locked = True
            log.info("NAV: EMERGENCY STOP. System LOCKED.")
        else:
            log.info("NAV: Stopped.")
        self.bridge.send_command("STOP")

    def unlock(self):
        self.emergency_locked = False
        log.info("NAV: System Unlocked.")

    def set_home(self, pos):
        """Set the home position for geofence checks."""
        self.home_position = pos

    # ── RTL ──────────────────────────────────────────────────────────
    def engage_rtl(self, home_pos=None):
        if len(self.history_path) < 2:
            if home_pos:
                log.warning("NAV: No history. Heading straight to Home.")
                self.set_waypoints([{"lat": home_pos["lat"], "lng": home_pos["lng"], "action": "end"}])
                self.start()
                return True
            log.error("NAV: Cannot engage RTL. No home reference.")
            return False

        log.info(f"NAV: RTL Engaged. Tracing back {len(self.history_path)} breadcrumbs...")
        traceback = [{"lat": p["lat"], "lng": p["lng"], "action": "move"} for p in reversed(self.history_path)]
        if home_pos:
            traceback.append({"lat": home_pos["lat"], "lng": home_pos["lng"], "action": "end"})
        else:
            traceback[-1]["action"] = "end"

        self.set_waypoints(traceback)
        self.start()
        self.history_path = []
        return True

    # ── Pure Pursuit Look-Ahead ──────────────────────────────────────
    def _pure_pursuit_target(self, curr_lat, curr_lng, target, velocity):
        """Compute look-ahead point on the path segment for Pure Pursuit."""
        cfg = nav_config.get_config()

        # Adaptive look-ahead: faster → look further ahead
        base_L = cfg["lookahead_distance"]
        speed_gain = cfg["lookahead_speed_gain"]
        L = base_L + (velocity * speed_gain)
        L = max(cfg["min_lookahead"], min(cfg["max_lookahead"], L))

        dist = haversine_distance(curr_lat, curr_lng, target["lat"], target["lng"])

        if dist <= L:
            # Close enough: aim directly at waypoint
            return target["lat"], target["lng"], 0.0, "DIRECT"

        # Project look-ahead point along the line from current pos to target
        ratio = L / dist if dist > 0 else 1.0
        look_lat = curr_lat + ratio * (target["lat"] - curr_lat)
        look_lng = curr_lng + ratio * (target["lng"] - curr_lng)

        # Cross-track error: perpendicular distance from rover to planned path
        # Simplified: use the segment from previous WP to current target
        xte = 0.0
        if self.current_wp_index > 0 and self.current_wp_index < len(self.waypoints):
            prev_wp = self.waypoints[self.current_wp_index - 1]
            xte = self._cross_track_error(
                curr_lat, curr_lng,
                prev_wp["lat"], prev_wp["lng"],
                target["lat"], target["lng"],
            )

        return look_lat, look_lng, xte, "PURE_PURSUIT"

    def _cross_track_error(self, lat, lng, wp1_lat, wp1_lng, wp2_lat, wp2_lng):
        """Signed cross-track error in meters (positive = right of path)."""
        # Angular distances
        d13 = haversine_distance(wp1_lat, wp1_lng, lat, lng) / 6371000.0
        brg13 = math.radians(calculate_bearing(wp1_lat, wp1_lng, lat, lng))
        brg12 = math.radians(calculate_bearing(wp1_lat, wp1_lng, wp2_lat, wp2_lng))
        xte = math.asin(math.sin(d13) * math.sin(brg13 - brg12)) * 6371000.0
        return round(xte, 2)

    # ── Adaptive Turn Speed ──────────────────────────────────────────
    def _get_turn_speed(self, brg_error):
        cfg = nav_config.get_config()
        abs_err = abs(brg_error)
        if abs_err > 90:
            return cfg["turn_speed_max"]
        elif abs_err > 30:
            ratio = abs_err / 90.0
            return int(cfg["turn_speed_min"] + ratio * (cfg["turn_speed_max"] - cfg["turn_speed_min"]))
        else:
            return cfg["turn_speed_min"]

    # ── Terrain-Aware Speed ──────────────────────────────────────────
    def _get_target_speed(self, dist, pitch_deg):
        cfg = nav_config.get_config()

        # Distance-based deceleration
        if dist < cfg["decel_radius"]:
            speed = cfg["move_speed_min"] + (dist / cfg["decel_radius"]) * (cfg["move_speed_max"] - cfg["move_speed_min"])
        else:
            speed = cfg["move_speed_max"]

        # Pitch penalty
        pitch_penalty = abs(pitch_deg) * cfg["pitch_speed_gain"]
        speed = max(cfg["move_speed_min"], speed - pitch_penalty)

        return speed

    # ══════════════════════════════════════════════════════════════════
    #  MAIN UPDATE LOOP — called every tick from Sensors Node
    # ══════════════════════════════════════════════════════════════════
    def update(self, current_pos, detections=None):
        """
        Args:
            current_pos: fused data dict from SensorFusion.update()
            detections:  list of detection dicts from perception (optional)
        """
        cfg = nav_config.get_config()
        now = time.time()
        dt = now - self.last_update_time if self.last_update_time > 0 else 0.05
        self.last_update_time = now

        curr_lat = current_pos.get("lat")
        curr_lng = current_pos.get("lng")
        curr_heading = current_pos.get("heading", 0.0)
        velocity = current_pos.get("velocity", 0.0)
        pitch = current_pos.get("pitch", 0.0)
        roll = current_pos.get("roll", 0.0)
        confidence = current_pos.get("confidence", {}).get("overall", 1.0)

        self.last_velocity = velocity

        # ── Always track history for RTL ──
        if curr_lat is not None and curr_lng is not None:
            if len(self.history_path) == 0 or haversine_distance(
                curr_lat, curr_lng, self.history_path[-1]["lat"], self.history_path[-1]["lng"]
            ) > 1.0:
                self.history_path.append({"lat": curr_lat, "lng": curr_lng})
                if len(self.history_path) > 500:
                    self.history_path.pop(0)

        # ── Not active: return minimal status ──
        if not self.active or self.emergency_locked:
            self.status["active"] = False
            self.status["state"] = "LOCKED" if self.emergency_locked else "IDLE"
            return self.status

        if self.current_wp_index >= len(self.waypoints):
            log.info("NAV: Mission Complete.")
            self.stop()
            return self.status

        if curr_lat is None or curr_lng is None:
            self._log_blocked(
                "NO_POSITION",
                now,
                f"gps_fix={current_pos.get('fix', False)} gps_connected={current_pos.get('connected', False)}",
            )
            return self.status

        # ── Terrain safety check ──
        if abs(pitch) > cfg["max_safe_pitch"] or abs(roll) > cfg["max_safe_roll"]:
            log.critical(f"NAV: TERRAIN HAZARD — pitch={pitch:.1f}° roll={roll:.1f}°. Emergency stop!")
            self.bridge.send_command("STOP")
            self.state = "TERRAIN_HALT"
            self.status.update({"state": "TERRAIN_HALT", "active": True})
            return self.status

        # If terrain cleared after halt, resume
        if self.state == "TERRAIN_HALT":
            if abs(pitch) < cfg["max_safe_pitch"] - 5 and abs(roll) < cfg["max_safe_roll"] - 5:
                log.info("NAV: Terrain safe. Resuming.")
                self.state = "IDLE"
            else:
                self.bridge.send_command("STOP")
                return self.status

        # ── Geofence check ──
        if cfg["geofence_enabled"] and self.home_position:
            dist_from_home = haversine_distance(
                curr_lat, curr_lng, self.home_position["lat"], self.home_position["lng"]
            )
            if dist_from_home > cfg["geofence_radius"]:
                log.critical(f"NAV: GEOFENCE BREACH — {dist_from_home:.0f}m from home (limit: {cfg['geofence_radius']}m)")
                self.bridge.send_command("STOP")
                self.state = "GEOFENCE_BREACH"
                self.engage_rtl(self.home_position)
                return self.status

        # ── Human detection pause ──
        if cfg["human_pause_enabled"] and detections:
            humans = [d for d in detections if d.get("label") == "HUMAN"]
            if humans:
                self.last_human_detection_time = now
                if not self.human_pause_active:
                    log.warning("NAV: HUMAN DETECTED — pausing navigation.")
                    self.human_pause_active = True
                    self.bridge.send_command("STOP")

        if self.human_pause_active:
            if now - self.last_human_detection_time > cfg["human_pause_clear_time"]:
                log.info("NAV: Human cleared. Resuming navigation.")
                self.human_pause_active = False
            else:
                self.bridge.send_command("STOP")
                self.status.update({"state": "PAUSED_HUMAN", "active": True})
                return self.status

        # ── Sensor degradation speed limit ──
        gps_outage = current_pos.get("gps_outage", False)
        speed_limit_factor = 1.0
        if gps_outage:
            speed_limit_factor = 0.6  # Reduce speed in DR-only mode

        # ── Core navigation ──
        target = self.waypoints[self.current_wp_index]
        dist = haversine_distance(curr_lat, curr_lng, target["lat"], target["lng"])
        target_brg = calculate_bearing(curr_lat, curr_lng, target["lat"], target["lng"])
        brg_error = get_bearing_error(curr_heading, target_brg)

        # Pure Pursuit look-ahead
        look_lat, look_lng, xte, nav_mode = self._pure_pursuit_target(
            curr_lat, curr_lng, target, velocity
        )
        pp_brg = calculate_bearing(curr_lat, curr_lng, look_lat, look_lng)
        pp_error = get_bearing_error(curr_heading, pp_brg)

        # Track segment progress
        if self.last_fused_pos:
            moved = haversine_distance(self.last_fused_pos[0], self.last_fused_pos[1], curr_lat, curr_lng)
            self.segment_dist_traveled += moved
        else:
            self.segment_start_pos = (curr_lat, curr_lng)
            self.expected_segment_dist = dist
            self.segment_dist_traveled = 0.0
        self.last_fused_pos = (curr_lat, curr_lng)

        # ETA estimate
        eta = dist / velocity if velocity > 0.1 else 0
        progress = self.current_wp_index / max(1, len(self.waypoints))

        # ── Update status ──
        self.status.update({
            "active": True,
            "distance_to_wp": round(dist, 2),
            "target_bearing": round(target_brg, 1),
            "bearing_error": round(brg_error, 1),
            "cross_track_error": round(xte, 2),
            "current_wp": self.current_wp_index,
            "total_wps": len(self.waypoints),
            "velocity": round(velocity, 2),
            "pitch": round(pitch, 1),
            "eta_seconds": int(eta),
            "confidence": round(confidence, 2),
            "stuck_count": self.stuck_count,
            "mission_progress": round(progress, 2),
            "mode": nav_mode,
            "state": self.state,
        })
        self._log_progress(
            now,
            dist=dist,
            brg_error=brg_error,
            velocity=velocity,
            curr_lat=curr_lat,
            curr_lng=curr_lng,
        )

        # ── ACTION_WAIT state ──
        if self.state == "ACTION_WAIT":
            action = target.get("action", "move").lower()
            if action in ["scan", "rotate 360"]:
                if now - self.last_cmd_time < 4.5:
                    self.bridge.send_command("DRIVE", -160, 160)
                    return self.status
                else:
                    log.info(f"NAV: Scan complete for WP {self.current_wp_index}")
            elif action == "stop":
                if now - self.last_cmd_time < 5.0:
                    self.bridge.send_command("STOP")
                    return self.status
                else:
                    log.info("NAV: Stop pause complete.")

            self.current_wp_index += 1
            self.state = "IDLE"
            self.last_fused_pos = None
            self.pid.reset()
            return self.status

        # ── ARRIVAL CHECK (Dual-Factor) ──
        reached_gps = dist < cfg["arrival_radius"]
        reached_odo = (self.expected_segment_dist > 1.0) and (
            self.segment_dist_traveled > (self.expected_segment_dist - cfg["odometry_arrival_margin"])
        )

        if reached_gps or reached_odo:
            self.arrival_ticks += 1
            if self.arrival_ticks >= cfg["arrival_latch_ticks"]:
                log.info(f"NAV: [ARRIVED] WP {self.current_wp_index} (GPS:{reached_gps}, ODO:{reached_odo})")
                self.arrival_ticks = 0
                self.stuck_count = 0
                action = target.get("action", "move").lower()

                if action in ["scan", "rotate 360", "stop"]:
                    self.state = "ACTION_WAIT"
                    self.last_cmd_time = now
                    self.bridge.send_command("STOP")
                    return self.status
                elif action == "end":
                    log.critical("NAV: Mission SUCCESS. ALL WAYPOINTS REACHED.")
                    self.stop()
                    return self.status
                else:
                    self.current_wp_index += 1
                    self.state = "IDLE"
                    self.target_heading_lock = None
                    self.last_fused_pos = None
                    self.pid.reset()
                    return self.status
        else:
            self.arrival_ticks = 0

        # ── STUCK DETECTION ──
        if self.state == "MOVING" and velocity < 0.05:
            self.stuck_timer += dt
            if self.stuck_timer > cfg["stuck_timeout"]:
                self.stuck_count += 1
                log.error(f"NAV: STUCK DETECTED (attempt {self.stuck_count}/{cfg['stuck_max_retries']})")
                self.stuck_timer = 0.0

                if self.stuck_count >= cfg["stuck_max_retries"]:
                    log.critical("NAV: Max stuck retries. Aborting mission.")
                    self.stop()
                    return self.status

                # Non-blocking recovery: enter STUCK_RECOVERY state
                # instead of blocking with time.sleep()
                self.bridge.send_command("DRIVE", -150, -150)  # Reverse
                self.state = "STUCK_RECOVERY"
                self._recovery_start = time.time()
                self._recovery_duration = cfg["stuck_recovery_reverse_time"]
                return self.status
        else:
            self.stuck_timer = 0.0

        # ── STUCK RECOVERY (non-blocking) ──
        if self.state == "STUCK_RECOVERY":
            if time.time() - self._recovery_start >= self._recovery_duration:
                self.bridge.send_command("STOP")
                self.state = "IDLE"
                self.pid.reset()
                log.info("NAV: Stuck recovery complete. Re-evaluating.")
            return self.status

        # ── STATE MACHINE ──
        if self.state == "IDLE":
            self.target_heading_lock = target_brg
            if dist < cfg["turn_close_skip_dist"] or abs(brg_error) < cfg["turn_entry_error"]:
                self.state = "MOVING"
            else:
                log.info(f"NAV: Alignment Turn (Target: {self.target_heading_lock:.0f}°, Error: {brg_error:.1f}°)")
                self.state = "TURNING"

        elif self.state == "TURNING":
            t_head = self.target_heading_lock if self.target_heading_lock is not None else target_brg
            locked_error = get_bearing_error(curr_heading, t_head)
            if abs(locked_error) < cfg["turn_exit_error"] or dist < cfg["turn_close_skip_dist"]:
                log.info("NAV: Alignment Complete → MOVING")
                self.state = "MOVING"
                self.pid.reset()

        elif self.state == "MOVING":
            if dist > cfg["turn_reentry_min_dist"] and abs(brg_error) > cfg["turn_reentry_error"]:
                self.state = "TURNING"
                self.target_heading_lock = target_brg
                self.pid.reset()

        # ── EXECUTION ──
        if self.state == "TURNING":
            t_head = self.target_heading_lock if self.target_heading_lock is not None else target_brg
            locked_error = get_bearing_error(curr_heading, t_head)
            turn_speed = self._get_turn_speed(locked_error)

            # Firmware contract: positive error means the target is to the right.
            # A right spin is left-backward/right-forward.
            if locked_error > 0:
                l_speed, r_speed = -int(turn_speed), int(turn_speed)
            else:
                l_speed, r_speed = int(turn_speed), -int(turn_speed)
            self._log_cmd(now, "TURN", l_speed, r_speed, f"err={locked_error:.1f} dist={dist:.2f}")
            self.bridge.send_command("DRIVE", l_speed, r_speed)
            self.last_cmd_time = now

        elif self.state == "MOVING":
            target_speed = self._get_target_speed(dist, pitch) * speed_limit_factor

            # PID steering on Pure Pursuit error
            dt_pid = now - self.pid.last_time if self.pid.last_time else 0.05
            self.pid.last_time = now
            steer = int(self.pid.compute(pp_error, max(dt_pid, 0.01)))

            self.status["pid_output"] = round(steer, 1)

            base_speed = int(target_speed)
            l_speed = base_speed - steer
            r_speed = base_speed + steer

            # Forward drive is positive on both sides in the STM32 contract.
            l_speed = max(-255, min(255, l_speed))
            r_speed = max(-255, min(255, r_speed))

            self._log_cmd(now, "MOVE", l_speed, r_speed, f"steer={steer} dist={dist:.2f}")
            self.bridge.send_command("DRIVE", l_speed, r_speed)
            self.last_cmd_time = now

        # ── Record history for RTL ──
        if self.last_history_pos is None:
            self.history_path.append({"lat": curr_lat, "lng": curr_lng})
            self.last_history_pos = (curr_lat, curr_lng)
        else:
            h_dist = haversine_distance(
                self.last_history_pos[0], self.last_history_pos[1], curr_lat, curr_lng
            )
            if h_dist > 1.0:
                self.history_path.append({"lat": curr_lat, "lng": curr_lng})
                self.last_history_pos = (curr_lat, curr_lng)
                if len(self.history_path) > 1000:
                    self.history_path.pop(0)

        return self.status


# ── Singleton ──
_nav_engine = None

def get_nav_engine():
    global _nav_engine
    if _nav_engine is None:
        _nav_engine = NavigationEngine()
    return _nav_engine
