"""
========================================================
  YUVAAN — STM32 DIRECT COMMAND DIAGNOSTIC TOOL
  Run this ON THE PI directly to bypass the entire
  backend and talk straight to the STM32.
  
  Usage:
      python3 tools/stm32_command_test.py
  
  This will tell you EXACTLY where the command chain is broken.
========================================================
"""

import serial
import serial.tools.list_ports
import time
import sys
import threading

# -------------------------------------------------------
# STEP 1: AUTO-DETECT STM32 PORT
# -------------------------------------------------------
def find_stm32_port():
    """Scans all available serial ports and lists them."""
    ports = list(serial.tools.list_ports.comports())
    print("\n" + "="*55)
    print("  YUVAAN STM32 PORT SCANNER")
    print("="*55)
    
    if not ports:
        print("  [FAIL] No serial ports found at all!")
        print("         → Check USB cable.")
        print("         → Check: ls /dev/tty* | grep USB")
        return None

    print(f"  Found {len(ports)} port(s):")
    for i, p in enumerate(ports):
        print(f"    [{i}] {p.device}  — {p.description}")

    # Auto-select if only one
    if len(ports) == 1:
        print(f"\n  [AUTO] Only one port found. Using: {ports[0].device}")
        return ports[0].device

    # Ask user if multiple
    try:
        choice = int(input(f"\n  Select port [0-{len(ports)-1}]: "))
        return ports[choice].device
    except (ValueError, IndexError):
        print("  [FAIL] Invalid selection.")
        return None


# -------------------------------------------------------
# STEP 2: OPEN PORT & VERIFY TELEMETRY IS FLOWING
# -------------------------------------------------------
def verify_telemetry(port, baud=115200, timeout=5):
    """Opens the port and checks if the STM32 is sending anything back."""
    print(f"\n{'='*55}")
    print(f"  PHASE 1: VERIFYING STM32 IS ALIVE (port={port})")
    print(f"{'='*55}")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Let the port settle
        
        print(f"  [OK] Port opened successfully.")
        print(f"  Waiting {timeout}s for STM32 telemetry...\n")
        
        lines_received = 0
        start = time.time()
        
        while time.time() - start < timeout:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    lines_received += 1
                    print(f"    RX [{lines_received:03d}]: {line}")
        
        print()
        if lines_received > 0:
            print(f"  [PASS] STM32 is alive. Received {lines_received} lines in {timeout}s.")
        else:
            print(f"  [FAIL] STM32 sent NOTHING in {timeout}s!")
            print(f"         → Is the STM32 powered on?")
            print(f"         → Is the firmware flashed correctly?")
            print(f"         → Try re-flashing STM32_Rover_Main.ino")
            ser.close()
            return None
        
        return ser
        
    except serial.SerialException as e:
        print(f"  [FAIL] Cannot open port '{port}': {e}")
        print(f"         → Try: sudo chmod 777 {port}")
        print(f"         → Or add user to dialout: sudo usermod -aG dialout $USER")
        return None


# -------------------------------------------------------
# STEP 3: SEND A COMMAND AND CHECK RESPONSE
# -------------------------------------------------------
def send_and_verify(ser, command, wait=4):
    """Sends one command and prints every line that comes back."""
    print(f"\n  → Sending: '{command}'")
    ser.write(command.encode())
    ser.flush()
    
    print(f"  ↓ Waiting {wait}s for response...")
    lines = []
    start = time.time()
    while time.time() - start < wait:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                lines.append(line)
                print(f"    STM32 → '{line}'")
        time.sleep(0.02)
    
    if not lines:
        print("    [FAIL] No response from STM32 after sending command!")
        return False
    
    # Check for expected keywords
    full_response = ' '.join(lines).upper()
    if 'COMPLETE' in full_response or 'MOVING' in full_response or 'TURNING' in full_response:
        print("    [PASS] STM32 acknowledged and executed the command!")
        return True
    else:
        print("    [WARN] Got response, but no execution keyword (MOVING/COMPLETE).")
        print("           This may mean the command was parsed but motors did nothing.")
        return False


# -------------------------------------------------------
# STEP 4: FULL DIAGNOSTIC SEQUENCE
# -------------------------------------------------------
def run_diagnostics(ser):
    print(f"\n{'='*55}")
    print("  PHASE 2: COMMAND PIPELINE TESTS")
    print(f"{'='*55}")
    
    # Test 1: STOP command (safe, always works if STM32 is alive)
    print("\n[TEST 1] Sending STOP (safe baseline test)")
    send_and_verify(ser, "$STOP,0,0*\n", wait=2)
    
    input("\n  [!] Press ENTER to test DRIVE (motors will spin briefly)...")
    
    # Test 2: DRIVE (non-blocking, motors spin for 0.5s then auto-stop)
    print("\n[TEST 2] Sending DRIVE (L=100, R=100 — both motors forward briefly)")
    ser.write(b"$DRIVE,100,100*\n")
    ser.flush()
    time.sleep(0.6)  # Let it run for 0.5s
    ser.write(b"$STOP,0,0*\n")  # Send STOP
    ser.flush()
    print("  Sent DRIVE then STOP. Did the motors physically spin?")
    print("  If YES → Serial pipe is working!")
    print("  If NO  → Check motor driver wiring / PWM pins in Config.h")
    
    input("\n  [!] Press ENTER to test MOVE (rover will physically move ~0.5m)...")
    
    # Test 3: MOVE (blocking — the key test)
    print("\n[TEST 3] Sending MOVE 0.5m (rover should physically move forward)")
    success = send_and_verify(ser, "$MOVE,0.5,0*\n", wait=8)
    if success:
        print("  [RESULT] MOVE COMMAND IS WORKING!")
    else:
        print("  [RESULT] MOVE COMMAND FAILED.")
        print("           → The command is being received but immediately aborted.")
        print("           → Most likely cause: Serial noise breaking the movement loop.")
        print("           → FIX: Re-flash with the updated STM32_Rover_Main.ino")
        print("                  (the noise-rejection fix in executeMove)")

    input("\n  [!] Press ENTER to test TURN 90 degrees (right turn)...")
    
    # Test 4: TURN
    print("\n[TEST 4] Sending TURN 90 degrees")
    success = send_and_verify(ser, "$TURN,90,0*\n", wait=8)
    if success:
        print("  [RESULT] TURN COMMAND IS WORKING!")
    else:
        print("  [RESULT] TURN COMMAND FAILED (same noise issue likely).")

    print(f"\n{'='*55}")
    print("  DIAGNOSTICS COMPLETE")
    print(f"{'='*55}")
    print("""
  SUMMARY:
  If TESTS 1-2 pass (STOP/DRIVE work) but TESTS 3-4 fail (MOVE/TURN don't):
    → The problem is confirmed: Serial noise (newlines) is aborting the
      blocking movement loops on the STM32.
    → ACTION: Re-flash STM32_Rover_Main.ino with the noise-rejection fix.

  If ALL tests fail (no response at all):
    → Serial port connection is broken or firmware is not flashed.
    → ACTION: Re-flash the firmware. Check USB-to-UART wiring.

  If tests pass here but GCS commands don't work:
    → The backend bridge.py is not correctly reaching the STM32.
    → ACTION: Run backend and check bridge logs. Verify serial port name
      in core/bridge.py matches the actual port (e.g., /dev/ttyUSB0).
""")


# -------------------------------------------------------
# MAIN
# -------------------------------------------------------
if __name__ == "__main__":
    print("\n  YUVAAN STM32 DIAGNOSTIC — Bypasses all backend code.")
    print("  Run this DIRECTLY on the Pi to find the exact failure point.")
    
    port = find_stm32_port()
    if not port:
        sys.exit(1)
    
    ser = verify_telemetry(port)
    if not ser:
        sys.exit(1)
    
    try:
        run_diagnostics(ser)
    except KeyboardInterrupt:
        print("\n  [ABORT] User cancelled diagnostics.")
    finally:
        ser.close()
        print("  Port closed. Goodbye.")
