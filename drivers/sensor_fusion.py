"""
YUVAAN Sensor Fusion Engine — drivers/sensor_fusion.py

Combines GPS (absolute position), IMU (relative orientation), Compass (absolute heading),
and Wheel Encoders (odometry) into a single fused state vector.

Key features:
  - Velocity-adaptive complementary filter for heading (gyro + compass + encoder heading)
  - Encoder-based differential heading as a third independent heading source
  - Pitch-corrected dead reckoning for sloped terrain
  - GPS outage detection with pure dead-reckoning fallback
  - Sensor confidence metrics for downstream decision-making
  - Position persistence across reboots
"""

import time
import math
import logging
import json
import os

from core import nav_config

log = logging.getLogger("YUVAAN.FUSION")


class SensorFusion:
    def __init__(self):
        # ── Fused State ──
        self.fused_lat = None
        self.fused_lng = None
        self.fused_heading = 0.0
        self.velocity = 0.0

        # ── Encoder Odometry State ──
        self.last_l_enc = 0
        self.last_r_enc = 0
        self.TICKS_PER_METER = 1355.0
        self.encoder_heading = 0.0  # Accumulated heading from differential encoders

        # ── GPS Outage Tracking ──
        self.last_gps_fix_time = 0.0
        self.gps_outage_active = False
        self.gps_reacquire_start = 0.0  # When GPS returned after an outage

        # ── Confidence Metrics ──
        self.confidence = {
            "gps": 0.0,
            "heading": 0.0,
            "position": 0.0,
            "overall": 0.0,
        }

        # ── Sensor Health Flags ──
        self.sensor_health = {
            "gps_connected": False,
            "gps_fix": False,
            "imu_connected": False,
            "compass_connected": False,
            "encoders_connected": False,
        }

        # ── Persistence ──
        self.persistence_file = "logs/last_pos.json"
        self._load_persistence()

        self.last_update = time.time()
        self._last_save_time = 0.0

        # ── Fallback base origin for no-GPS scenarios ──
        self.base_lat = float(os.environ.get("YUVAAN_BASE_LAT", "18.4485"))
        self.base_lng = float(os.environ.get("YUVAAN_BASE_LNG", "77.4562"))

    # ──────────────────────────────────────────────────────────────────────
    #  MAIN UPDATE — Called every tick (~20 Hz) from Sensors Node
    # ──────────────────────────────────────────────────────────────────────
    def update(self, gps_data, imu_data, compass_heading, bridge_data=None):
        """
        Args:
            gps_data:        {'lat', 'lng', 'fix', 'sats', 'connected', ...}
            imu_data:        {'yaw_rate', 'pitch', 'roll', 'connected', ...}
            compass_heading: float (degrees, 0-360, clockwise from North)
            bridge_data:     {'encoders': {'left', 'right'}, 'connected', ...}

        Returns:
            dict with fused lat/lng/heading/velocity/pitch/roll/confidence/...
        """
        now = time.time()
        dt = min(now - self.last_update, 0.5)  # Cap dt to prevent startup spikes
        self.last_update = now

        cfg = nav_config.get_config()
        wheelbase = cfg["wheelbase"]

        # ── Update sensor health flags ──
        self.sensor_health["gps_connected"] = gps_data.get("connected", False)
        self.sensor_health["gps_fix"] = gps_data.get("fix", False)
        self.sensor_health["imu_connected"] = imu_data.get("connected", True)
        self.sensor_health["compass_connected"] = compass_heading is not None
        self.sensor_health["encoders_connected"] = (
            bridge_data is not None and bridge_data.get("connected", False)
        )

        # ════════════════════════════════════════════════════════════════
        #  1. ENCODER ODOMETRY + DIFFERENTIAL HEADING
        # ════════════════════════════════════════════════════════════════
        dist_moved = 0.0
        encoder_heading_delta = 0.0

        if bridge_data and bridge_data.get("connected"):
            curr_l = bridge_data["encoders"].get("left", 0)
            curr_r = bridge_data["encoders"].get("right", 0)

            d_l = (curr_l - self.last_l_enc) / self.TICKS_PER_METER
            d_r = (curr_r - self.last_r_enc) / self.TICKS_PER_METER

            # Filter out encoder glitches (large jumps)
            if abs(d_l) > 5.0 or abs(d_r) > 5.0:
                d_l = 0.0
                d_r = 0.0

            # SIGNED ODOMETRY: Forward is -L, +R in our wiring
            dist_moved = (-d_l + d_r) / 2.0

            # DIFFERENTIAL HEADING: Δθ = (d_right - d_left) / wheelbase
            # In our wiring: right encoder is positive-forward, left is negative-forward
            # So effective: d_right_fwd = d_r, d_left_fwd = -d_l
            d_left_fwd = -d_l
            d_right_fwd = d_r
            encoder_heading_delta = math.degrees(
                (d_right_fwd - d_left_fwd) / wheelbase
            ) if wheelbase > 0 else 0.0

            # Accumulate encoder-based heading
            self.encoder_heading = (self.encoder_heading + encoder_heading_delta) % 360.0

            # Instantaneous velocity
            inst_velocity = dist_moved / dt if dt > 0 else 0
            self.velocity = (0.8 * self.velocity) + (0.2 * abs(inst_velocity))

            self.last_l_enc = curr_l
            self.last_r_enc = curr_r

            # PITCH-CORRECTED DEAD RECKONING
            pitch_rad = math.radians(imu_data.get("pitch", 0))
            horizontal_dist = dist_moved * math.cos(pitch_rad)

            if self.fused_lat is not None and abs(horizontal_dist) > 0.001:
                rad = math.radians(self.fused_heading)
                delta_lat = (horizontal_dist * math.cos(rad)) / 111111.0
                delta_lng = (horizontal_dist * math.sin(rad)) / (
                    111111.0 * math.cos(math.radians(self.fused_lat))
                )
                self.fused_lat += delta_lat
                self.fused_lng += delta_lng

        # ════════════════════════════════════════════════════════════════
        #  2. HEADING FUSION (Three-Source Complementary Filter)
        # ════════════════════════════════════════════════════════════════
        if imu_data.get("connected", True):
            gyro_z = imu_data.get("yaw_rate", 0)
            # Gyro Z is positive counter-clockwise; compass is clockwise (N=0, E=90)
            # Subtract gyro_z to align with compass convention
            predicted_heading = self.fused_heading - (gyro_z * dt)

            # Velocity-adaptive weights for three-source fusion
            is_moving = self.velocity > 0.15

            if is_moving:
                w_gyro = cfg["gyro_weight_moving"]
                w_compass = cfg["compass_weight_moving"]
                w_encoder = cfg["encoder_heading_weight_moving"]
            else:
                w_gyro = cfg["gyro_weight_stationary"]
                w_compass = cfg["compass_weight_stationary"]
                w_encoder = cfg["encoder_heading_weight_stationary"]

            # Normalize weights to sum to 1.0
            w_total = w_gyro + w_compass + w_encoder
            if w_total > 0:
                w_gyro /= w_total
                w_compass /= w_total
                w_encoder /= w_total

            # Wrap-safe blending: compass vs predicted
            diff_compass = compass_heading - predicted_heading
            if diff_compass > 180:
                diff_compass -= 360
            elif diff_compass < -180:
                diff_compass += 360

            # Wrap-safe blending: encoder heading vs predicted
            diff_encoder = self.encoder_heading - predicted_heading
            if diff_encoder > 180:
                diff_encoder -= 360
            elif diff_encoder < -180:
                diff_encoder += 360

            # Final blended heading: gyro prediction + compass correction + encoder correction
            self.fused_heading = (
                predicted_heading
                + (w_compass * diff_compass)
                + (w_encoder * diff_encoder)
            )
        else:
            # IMU dead — fall back to compass only
            self.fused_heading = compass_heading

        self.fused_heading %= 360

        # ════════════════════════════════════════════════════════════════
        #  3. GPS POSITION ANCHORING (with outage detection)
        # ════════════════════════════════════════════════════════════════
        gps_has_fix = gps_data.get("connected", False) and gps_data.get("fix", False)

        if gps_has_fix:
            self.last_gps_fix_time = now

            # Were we in an outage that just cleared?
            if self.gps_outage_active:
                log.info("FUSION: GPS fix re-acquired after outage. Ramping trust.")
                self.gps_outage_active = False
                self.gps_reacquire_start = now

            if self.fused_lat is None:
                # First fix — snap to GPS
                self.fused_lat = gps_data["lat"]
                self.fused_lng = gps_data["lng"]
                log.info(f"FUSION: Initial GPS fix: {self.fused_lat:.6f}, {self.fused_lng:.6f}")
            else:
                # Weighted pull toward GPS
                is_actually_moving = self.velocity > 0.15 or abs(dist_moved) > 0.01

                # Ramp GPS trust back slowly after an outage
                time_since_reacquire = now - self.gps_reacquire_start
                ramp_time = cfg["gps_reacquire_ramp_time"]
                reacquire_factor = min(1.0, time_since_reacquire / ramp_time) if self.gps_reacquire_start > 0 else 1.0

                if is_actually_moving:
                    base_weight = 0.2 if gps_data.get("sats", 0) > 6 else 0.05
                    weight = base_weight * reacquire_factor
                else:
                    # Stationary: very slow pull to average out jitter
                    weight = 0.02 * reacquire_factor

                self.fused_lat = (weight * gps_data["lat"]) + ((1 - weight) * self.fused_lat)
                self.fused_lng = (weight * gps_data["lng"]) + ((1 - weight) * self.fused_lng)

        else:
            # No GPS fix — check for outage
            time_without_fix = now - self.last_gps_fix_time if self.last_gps_fix_time > 0 else 0

            if time_without_fix > cfg["gps_outage_threshold"] and not self.gps_outage_active:
                self.gps_outage_active = True
                log.warning(f"FUSION: GPS OUTAGE — no fix for {time_without_fix:.1f}s. Pure dead-reckoning mode.")

            if self.fused_lat is None:
                # No GPS and no prior position — use base fallback
                self.fused_lat = self.base_lat
                self.fused_lng = self.base_lng
                log.warning(f"FUSION: No GPS. Using Dead-Reckoning Base: {self.fused_lat}, {self.fused_lng}")

        # ════════════════════════════════════════════════════════════════
        #  4. CONFIDENCE METRICS
        # ════════════════════════════════════════════════════════════════
        self._update_confidence(gps_data, imu_data, compass_heading, now)

        # ════════════════════════════════════════════════════════════════
        #  5. PERSISTENCE (save every ~2 seconds)
        # ════════════════════════════════════════════════════════════════
        if now - self._last_save_time > 2.0:
            self._save_persistence()
            self._last_save_time = now

        # ════════════════════════════════════════════════════════════════
        #  6. RETURN FUSED STATE
        # ════════════════════════════════════════════════════════════════
        return {
            "lat": self.fused_lat,
            "lng": self.fused_lng,
            "heading": round(self.fused_heading, 2),
            "velocity": round(self.velocity, 2),
            "fix": gps_data.get("fix", False),
            "sats": gps_data.get("sats", 0),
            "pitch": imu_data.get("pitch", 0),
            "roll": imu_data.get("roll", 0),
            "gps_connected": gps_data.get("connected", False),
            "connected": gps_data.get("connected", False),  # Alias for frontend compat (checks gps.connected)
            "imu_connected": imu_data.get("connected", False),
            "mag_connected": self.sensor_health.get("compass_connected", False),
            "encoders_connected": self.sensor_health.get("encoders_connected", False),
            "gps_outage": self.gps_outage_active,
            "confidence": dict(self.confidence),
            "encoder_heading": round(self.encoder_heading, 2),
        }

    # ──────────────────────────────────────────────────────────────────────
    #  CONFIDENCE COMPUTATION
    # ──────────────────────────────────────────────────────────────────────
    def _update_confidence(self, gps_data, imu_data, compass_heading, now):
        """Compute per-sensor and overall confidence scores (0.0–1.0)."""

        # GPS confidence: based on fix, sat count
        if gps_data.get("fix", False) and gps_data.get("connected", False):
            sats = gps_data.get("sats", 0)
            self.confidence["gps"] = min(1.0, max(0.1, sats / 12.0))
        elif self.gps_outage_active:
            self.confidence["gps"] = 0.0
        else:
            self.confidence["gps"] = 0.05  # Connected but no fix

        # Heading confidence: agreement between compass and fused heading
        heading_diff = abs(compass_heading - self.fused_heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        # Good agreement (<10°) = 1.0, poor (>60°) = 0.2
        self.confidence["heading"] = max(0.2, 1.0 - (heading_diff / 60.0))

        if not imu_data.get("connected", True):
            self.confidence["heading"] *= 0.3  # Severely degraded without gyro

        # Position confidence: composite
        if self.gps_outage_active:
            # Degrades over time without GPS
            outage_time = now - self.last_gps_fix_time
            self.confidence["position"] = max(0.1, 1.0 - (outage_time / 60.0))
        else:
            self.confidence["position"] = self.confidence["gps"] * 0.7 + 0.3

        # Overall: minimum of components
        self.confidence["overall"] = min(
            self.confidence["gps"],
            self.confidence["heading"],
            self.confidence["position"],
        )

    # ──────────────────────────────────────────────────────────────────────
    #  PERSISTENCE
    # ──────────────────────────────────────────────────────────────────────
    def _load_persistence(self):
        if os.path.exists(self.persistence_file):
            try:
                with open(self.persistence_file, "r") as f:
                    data = json.load(f)
                    self.fused_lat = data.get("lat")
                    self.fused_lng = data.get("lng")
                    self.fused_heading = data.get("heading", 0.0)
                    log.info(f"FUSION: Loaded persisted position: {self.fused_lat}, {self.fused_lng}")
            except Exception:
                pass

    def _save_persistence(self):
        if self.fused_lat is not None:
            try:
                os.makedirs(os.path.dirname(self.persistence_file) or ".", exist_ok=True)
                with open(self.persistence_file, "w") as f:
                    json.dump(
                        {"lat": self.fused_lat, "lng": self.fused_lng, "heading": self.fused_heading},
                        f,
                    )
            except Exception:
                pass


def get_fusion_engine():
    return SensorFusion()
