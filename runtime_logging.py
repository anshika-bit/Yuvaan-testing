import json
import logging
import os
import sys
import time
from collections import deque
from datetime import datetime


PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(PROJECT_ROOT, "logs")
RUNTIME_LOG_PATH = os.path.join(LOG_DIR, "backend_events.jsonl")

_CONSOLE_FORMAT = "[%(asctime)s] %(levelname)s | %(name)s | %(message)s"
_CONSOLE_DATEFMT = "%H:%M:%S"


class JsonlRuntimeLogHandler(logging.Handler):
    """Write structured runtime logs to a shared JSONL file."""

    def __init__(self, log_path):
        super().__init__()
        self.log_path = log_path
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

    def emit(self, record):
        try:
            entry = {
                "id": f"{int(record.created * 1000)}-{record.process}-{record.thread}",
                "timestamp": datetime.fromtimestamp(record.created).strftime("%H:%M:%S"),
                "level": record.levelname,
                "logger": record.name,
                "message": record.getMessage(),
                "created": record.created,
                "process": record.process,
            }
            with open(self.log_path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(entry, ensure_ascii=True) + "\n")
        except Exception:
            self.handleError(record)


def configure_runtime_logging(console=True):
    """Configure the shared YUVAAN logger tree for the current process."""
    os.makedirs(LOG_DIR, exist_ok=True)

    yuvaan_logger = logging.getLogger("YUVAAN")
    yuvaan_logger.setLevel(logging.INFO)
    yuvaan_logger.propagate = False

    for handler in list(yuvaan_logger.handlers):
        yuvaan_logger.removeHandler(handler)
        try:
            handler.close()
        except Exception:
            pass

    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(logging.Formatter(_CONSOLE_FORMAT, datefmt=_CONSOLE_DATEFMT))
        yuvaan_logger.addHandler(console_handler)

    file_handler = JsonlRuntimeLogHandler(RUNTIME_LOG_PATH)
    yuvaan_logger.addHandler(file_handler)
    return yuvaan_logger


def reset_runtime_logs():
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(RUNTIME_LOG_PATH, "w", encoding="utf-8") as handle:
        handle.write("")


def get_recent_runtime_logs(limit=50):
    if limit <= 0 or not os.path.isfile(RUNTIME_LOG_PATH):
        return []

    recent = deque(maxlen=limit)
    with open(RUNTIME_LOG_PATH, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                recent.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return list(recent)


def build_mission_control_screen(*, tick, bridge_data, gps_raw, fused_data, nav, nav_status, recent_logs):
    """Build the backend terminal HUD."""

    def fmt_number(value, digits=1, fallback="n/a"):
        if value is None:
            return fallback
        try:
            return f"{float(value):.{digits}f}"
        except Exception:
            return fallback

    def fmt_coord(value):
        if value is None:
            return "n/a"
        try:
            return f"{float(value):.6f}"
        except Exception:
            return "n/a"

    link_online = bridge_data.get("connected", False)
    link_state = "ONLINE" if link_online else "OFFLINE"
    hw_mode = str(bridge_data.get("mode", "UNKNOWN")).upper()

    last_update = bridge_data.get("last_update", 0) or 0
    if last_update > 0:
        rx_status = f"{max(0.0, time.time() - last_update):.1f}s"
    else:
        rx_status = "NEVER"

    battery = bridge_data.get("battery", {})
    motors = bridge_data.get("motors", {})
    fused_heading = fused_data.get("heading")
    gps_fix = "FIXED" if gps_raw.get("fix") else "SEARCHING"

    nav_state = nav_status.get("state", "IDLE")
    recent_section = []
    for event in recent_logs[-5:]:
        logger_name = event.get("logger", "YUVAAN")
        source = logger_name.split(".", 1)[-1] if "." in logger_name else logger_name
        recent_section.append(
            f" [{event.get('timestamp', '--:--:--')}] {event.get('level', 'INFO'):<7} {source:<12} {event.get('message', '')}"
        )

    if not recent_section:
        recent_section.append(" [--:--:--] INFO    SYSTEM       Waiting for backend events...")

    lines = [
        "=" * 78,
        f" YUVAAN MISSION CONTROL | T+{tick / 20.0:6.1f}s | {datetime.now().strftime('%H:%M:%S')}",
        "=" * 78,
        f" [CONTROL] MODE: {hw_mode:<7} | LINK: {link_state:<7} | LAST RX: {rx_status:<8} | NAV: {nav_state}",
        f" [POWER]   BATT: {battery.get('percent', 0):>3}% ({fmt_number(battery.get('voltage'), 2, '0.00')}V) | "
        f"VEL: {fmt_number(motors.get('velocity'), 2, '0.00')} m/s | RPM: {motors.get('rpm', 0):>4}",
        f" [POSE]    LAT: {fmt_coord(fused_data.get('lat'))} | LNG: {fmt_coord(fused_data.get('lng'))} | "
        f"HDG: {fmt_number(fused_heading, 1, '0.0')} deg",
        f" [GPS]     FIX: {gps_fix:<9} | SATS: {gps_raw.get('sats', 0):>2} | RAW LAT/LNG: "
        f"{fmt_coord(gps_raw.get('lat'))}, {fmt_coord(gps_raw.get('lng'))}",
    ]

    if nav.active:
        total_wps = len(getattr(nav, "waypoints", []))
        wp_index = getattr(nav, "current_wp_index", 0) + 1 if total_wps else 0
        lines.append(
            f" [MISSION] STATE: {nav_state:<12} | WP: {wp_index}/{total_wps} | "
            f"DIST: {fmt_number(nav_status.get('distance_to_wp'), 1, '0.0')} m | "
            f"ERR: {fmt_number(nav_status.get('bearing_error'), 1, '0.0')} deg"
        )
    else:
        idle_state = "LOCKED (E-STOP)" if getattr(nav, "emergency_locked", False) else "IDLE (WAITING)"
        lines.append(f" [MISSION] STATUS: {idle_state}")

    lines.append("-" * 78)
    lines.append(" RECENT EVENTS")
    lines.extend(recent_section)
    lines.append("-" * 78)
    lines.append(" Ctrl+C to shutdown")
    return "\n".join(lines)
