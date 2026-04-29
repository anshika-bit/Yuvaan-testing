"""
YUVAAN Navigation Configuration — core/nav_config.py

Centralized, live-reloadable configuration for all navigation parameters.
Loaded at startup. Can be updated at runtime via POST /api/navigation/config.
"""

import json
import os
import logging
import threading

log = logging.getLogger("YUVAAN.NAVCONFIG")

_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "nav_config.json")

# ── Default Configuration ──────────────────────────────────────────────
_DEFAULTS = {
    # ── PID Steering ──
    "pid_kp": 2.0,          # Proportional gain for bearing error → steering correction
    "pid_ki": 0.05,         # Integral gain — eliminates steady-state error (wind, terrain)
    "pid_kd": 0.8,          # Derivative gain — dampens oscillation & overshoot
    "pid_integral_max": 50,  # Anti-windup clamp for integral term

    # ── Pure Pursuit ──
    "lookahead_distance": 2.0,   # meters — look-ahead circle radius
    "min_lookahead": 1.0,        # clamp: minimum look-ahead
    "max_lookahead": 5.0,        # clamp: maximum look-ahead
    "lookahead_speed_gain": 2.0, # scale look-ahead with velocity (L = base + vel * gain)

    # ── Speed Control ──
    "move_speed_max": 200,
    "move_speed_min": 130,
    "turn_speed_max": 180,
    "turn_speed_min": 100,
    "decel_radius": 4.0,         # meters — begin linear deceleration

    # ── Terrain ──
    "pitch_speed_gain": 0.5,     # reduce speed by this * abs(pitch_degrees)
    "max_safe_pitch": 25.0,      # degrees — emergency stop above this
    "max_safe_roll": 30.0,       # degrees — emergency stop above this

    # ── Arrival Detection ──
    "arrival_radius": 1.5,       # meters — GPS distance threshold
    "arrival_latch_ticks": 10,   # consecutive ticks needed to confirm arrival
    "odometry_arrival_margin": 0.5,  # meters — how close odo must be to expected dist

    # ── State Machine Thresholds ──
    "turn_entry_error": 45.0,    # degrees — start in TURNING if error exceeds this
    "turn_exit_error": 5.0,      # degrees — switch from TURNING → MOVING below this
    "turn_reentry_error": 60.0,  # degrees — from MOVING back to TURNING (catastrophic drift)
    "turn_reentry_min_dist": 5.0,# meters — only re-enter TURNING if far enough away
    "turn_close_skip_dist": 2.5, # meters — skip TURNING if very close to WP

    # ── Safety ──
    "stuck_timeout": 5.0,        # seconds — no encoder movement = stuck
    "stuck_recovery_reverse_time": 1.0,  # seconds to reverse during recovery
    "stuck_max_retries": 3,      # abort mission after this many consecutive stucks
    "human_pause_enabled": False,      # Set True to pause nav on human detection
    "human_pause_clear_time": 3.0,  # seconds of no detections before resuming
    "geofence_enabled": True,
    "geofence_radius": 200.0,    # meters from home position

    # ── Sensor Fusion Weights ──────────────────────────────────────────
    # TUNING GUIDE: The three weights (gyro + compass + encoder) are
    # auto-normalized to 1.0. To change the compass:gyro ratio while
    # keeping encoder fixed, just edit the two values below.
    #
    #   Current MOVING ratio  → compass 60%, gyro 40% (of non-encoder share)
    #   encoder = 0.15  |  remaining 0.85 split 60/40
    #   compass = 0.85 × 0.60 = 0.51
    #   gyro    = 0.85 × 0.40 = 0.34
    #
    # For a 50/50 split: compass=0.425, gyro=0.425
    # For a 70/30 split: compass=0.595, gyro=0.255
    # ──────────────────────────────────────────────────────────────────
    "wheelbase": 0.50,           # meters — distance between left and right wheel centers
    "encoder_heading_weight_moving": 0.15,
    "encoder_heading_weight_stationary": 0.05,
    "gyro_weight_moving": 0.34,          # ← 40% of non-encoder share
    "gyro_weight_stationary": 0.50,
    "compass_weight_moving": 0.51,       # ← 60% of non-encoder share
    "compass_weight_stationary": 0.45,
    "gps_outage_threshold": 5.0,     # seconds — pure DR mode after this
    "gps_reacquire_ramp_time": 3.0,  # seconds — slowly trust GPS again
}

# ── Runtime State ──────────────────────────────────────────────────────
_config = dict(_DEFAULTS)
_lock = threading.Lock()


def get_config():
    """Return a snapshot of the current configuration."""
    with _lock:
        return dict(_config)


def get(key, default=None):
    """Get a single config value."""
    with _lock:
        return _config.get(key, default)


def update_config(updates: dict):
    """Merge updates into the live config. Only known keys are accepted."""
    with _lock:
        changed = []
        for k, v in updates.items():
            if k in _DEFAULTS:
                old = _config[k]
                _config[k] = type(_DEFAULTS[k])(v)  # Cast to original type
                if old != _config[k]:
                    changed.append(f"{k}: {old} → {_config[k]}")
            else:
                log.warning(f"NAV_CONFIG: Unknown key '{k}' ignored.")
        if changed:
            log.info(f"NAV_CONFIG: Updated — {', '.join(changed)}")
        _save()


def reset_defaults():
    """Reset all values to defaults."""
    with _lock:
        _config.clear()
        _config.update(_DEFAULTS)
    _save()
    log.info("NAV_CONFIG: Reset to defaults.")


def _load():
    """Load persisted overrides from disk."""
    global _config
    try:
        if os.path.exists(_CONFIG_FILE):
            with open(_CONFIG_FILE, "r") as f:
                saved = json.load(f)
            with _lock:
                for k, v in saved.items():
                    if k in _DEFAULTS:
                        _config[k] = type(_DEFAULTS[k])(v)
            log.info(f"NAV_CONFIG: Loaded {len(saved)} overrides from {_CONFIG_FILE}")
    except Exception as e:
        log.warning(f"NAV_CONFIG: Could not load config file: {e}")


def _save():
    """Persist current config to disk."""
    try:
        os.makedirs(os.path.dirname(_CONFIG_FILE), exist_ok=True)
        with open(_CONFIG_FILE, "w") as f:
            json.dump(_config, f, indent=2)
    except Exception as e:
        log.warning(f"NAV_CONFIG: Could not save config: {e}")


# ── Auto-load on import ───────────────────────────────────────────────
_load()
