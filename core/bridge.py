import serial
import serial.tools.list_ports
import time
import threading
import logging
import os

log = logging.getLogger("YUVAAN.BRIDGE")

def _auto_detect_stm32_port():
    """Scans serial ports and returns the first STM32/CDC device found."""
    candidates = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/stm32_main"]
    for p in candidates:
        if os.path.exists(p):
            log.info(f"BRIDGE: Auto-detected STM32 port: {p}")
            return p
    # Fallback: scan all ports for CDC device
    for port in serial.tools.list_ports.comports():
        if "CDC" in (port.description or "") or "STM" in (port.description or "").upper():
            log.info(f"BRIDGE: Found STM32 by description: {port.device} — {port.description}")
            return port.device
    log.warning("BRIDGE: Could not auto-detect STM32 port. Defaulting to /dev/ttyACM0")
    return "/dev/ttyACM0"

class BridgeController:
    """
    Handles communication with the STM32 Motor Controller.
    Parses telemetry feedback (Encoders, Battery, Motors) and sends commands.
    """
    def __init__(self, port=None, baud=115200):
        port = port or _auto_detect_stm32_port()
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self._lock = threading.Lock()
        
        # Latest Telemetry Data
        self.telemetry = {
            "connected": False,
            "mode": "UNKNOWN",
            "battery": {"percent": 0, "voltage": 0.0},
            "motors": {"velocity": 0.0, "rpm": 0, "current": 0.0},
            "encoders": {"left": 0, "right": 0},
            "last_update": 0,
            "last_cmd": None,
            "last_cmd_ts": 0
        }
        
        self._last_enc_time = 0
        self._last_l_enc = 0
        self._last_r_enc = 0
        self.TICKS_PER_METER = 1355.0
        
        # Start background listener
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()

    def _listen_loop(self):
        while self.running:
            if not os.path.exists(self.port):
                self.telemetry["connected"] = False
                log.warning(f"BRIDGE: Port {self.port} not found. Retrying in 2s...")
                time.sleep(2)
                continue
                
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                log.info(f"BRIDGE: Connected to STM32 on {self.port}")
                self.telemetry["connected"] = True
                
                while self.running:
                    if self.ser.in_waiting > 0:
                        try:
                            # Protocol uses '*' as terminator. readline() might hang if \n is missing.
                            raw_data = self.ser.read_until(b'*')
                            if raw_data:
                                line = raw_data.decode('utf-8', errors='ignore').strip()
                                if line:
                                    self._parse_line(line)
                        except Exception as e:
                            log.error(f"BRIDGE: Parse error: {e}")
                    time.sleep(0.01)
            except Exception as e:
                log.error(f"BRIDGE: Serial error: {e}")
                self.telemetry["connected"] = False
                if self.ser: self.ser.close()
                time.sleep(2)

    def _parse_line(self, line):
        """
        Parses STM32 protocol:
        $BATT,85,24.5* -> Battery percentage, voltage
        $MOTOR,1.2,450,3.5* -> Velocity (m/s), RPM, Current (A)
        $ENC,1234,1250* -> Left, Right encoder ticks
        $MODE,AUT* -> Current STM32 Mode
        """
        line = line.strip()
        if not line: return

        try:
            # Handle protocol packets
            if line.startswith('$'):
                # Extract content between $ and *
                # Example: $MODE,AUT* -> MODE,AUT
                # Example: $MODE,AUT -> MODE,AUT
                content = line[1:].split('*')[0].strip()
                parts = content.split(',')
                header = parts[0]
            else:
                # Handle non-protocol lines (logs or status messages from STM32)
                if any(k in line for k in ["COMPLETE", "ERROR", ">>>", "MOVING", "TURNING"]):
                    log.debug(f"STM32 LOG: {line}")
                return

            with self._lock:
                if header == "BATT" and len(parts) >= 3:
                    self.telemetry["battery"]["percent"] = int(parts[1])
                    self.telemetry["battery"]["voltage"] = float(parts[2])
                elif header == "MOTOR" and len(parts) >= 4:
                    self.telemetry["motors"]["velocity"] = float(parts[1])
                    self.telemetry["motors"]["rpm"] = int(parts[2])
                    self.telemetry["motors"]["current"] = float(parts[3])
                elif header == "ENC" and len(parts) >= 3:
                    new_l = int(parts[1])
                    new_r = int(parts[2])
                    self.telemetry["encoders"]["left"] = new_l
                    self.telemetry["encoders"]["right"] = new_r
                    
                    now = time.time()
                    dt = now - self._last_enc_time
                    if self._last_enc_time > 0 and dt > 0:
                        # Handle Encoder Wrap-Around (Assume max jump of 10,000 ticks)
                        dl_ticks = new_l - self._last_l_enc
                        dr_ticks = new_r - self._last_r_enc
                        
                        if abs(dl_ticks) > 10000: dl_ticks = 0 # Ignore jump
                        if abs(dr_ticks) > 10000: dr_ticks = 0 # Ignore jump
                        
                        dl = dl_ticks / self.TICKS_PER_METER
                        dr = dr_ticks / self.TICKS_PER_METER
                        avg_dist = (dl + dr) / 2.0
                        velocity = avg_dist / dt
                        
                        # Apply low-pass filter to smooth velocity
                        self.telemetry["motors"]["velocity"] = (0.8 * self.telemetry["motors"]["velocity"]) + (0.2 * abs(velocity))
                        
                    self._last_enc_time = now
                    self._last_l_enc = new_l
                    self._last_r_enc = new_r
                elif header == "MODE" and len(parts) >= 2:
                    new_mode = parts[1].strip().upper()
                    # Only accept valid mode values — reject corrupted serial fragments
                    if new_mode in ("MAN", "AUT"):
                        if new_mode != self.telemetry["mode"]:
                            log.info(f"BRIDGE: Hardware mode changed to {new_mode}")
                        self.telemetry["mode"] = new_mode
                
                self.telemetry["last_update"] = time.time()
                
        except Exception as e:
            log.debug(f"BRIDGE: Error parsing line '{line}': {e}")

    def send_command(self, cmd_type, val1=0, val2=0):
        """
        Sends commands to STM32:
        $MOVE,distance,speed*
        $TURN,angle,speed*
        $SCAN,0,0*
        $STOP,0,0*
        """

        if not self.telemetry["connected"] or not self.ser:
            log.error("BRIDGE: Cannot send command, STM32 not connected.")
            return False
            
        try:
            # Normalize to integers to ensure STM32 parser compatibility
            v1, v2 = int(val1), int(val2)

            # MOTOR WIRING CORRECTION: The L/R drive channels are swapped
            # on the STM32, and one motor is physically inverted.
            # Transform: (v1, v2) → (-v2, v1) so the rest of the codebase
            # can use the intuitive convention (v1=left, v2=right, +=forward).
            if cmd_type == "DRIVE":
                v1, v2 = -v2, v1
            
            # NO NEWLINE: STM32 parser uses '*' as the strict terminator
            packet = f"${cmd_type},{v1},{v2}*"
            
            # DEDUPLICATION: Only log if command changed or values are different 
            # (Exception for DRIVE/MOVE which update frequently)
            last_cmd = self.telemetry.get("last_cmd", "")
            is_duplicate = (packet.strip() == last_cmd) and (cmd_type in ["STOP", "SCAN", "TURN"])
            
            with self._lock:
                self.ser.write(packet.encode())
                self.ser.flush()
                self.telemetry["last_cmd"] = packet.strip()
                self.telemetry["last_cmd_ts"] = time.time()
                
            if not is_duplicate:
                log.info(f"BRIDGE -> STM32: {packet.strip()}")

            return True
        except Exception as e:
            log.error(f"BRIDGE: Failed to send command: {e}")
            return False

    def get_telemetry(self):
        with self._lock:
            return self.telemetry.copy()

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

# Singleton accessor
_bridge = None

def get_bridge(port=None):
    global _bridge
    if _bridge is None:
        _bridge = BridgeController(port=port)  # port=None triggers auto-detection
    return _bridge

if __name__ == "__main__":
    # Test script
    logging.basicConfig(level=logging.INFO)
    bridge = get_bridge()
    try:
        while True:
            print(f"Telemetry: {bridge.get_telemetry()}")
            time.sleep(1)
    except KeyboardInterrupt:
        bridge.stop()