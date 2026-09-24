#!/usr/bin/env python3
"""
AMR USB Auto-Detection Script (Fixed)
=====================================
Dynamic serial probing to distinguish ESP32 and RPLiDAR.

Logic:
  - ESP32: Sends continuous telemetry 'PKT,...' or responds to 'PING'
  - RPLiDAR: Requires DTR pin toggle to spin motor, streams binary sync bytes (0xA5)
"""

import os
import sys
import glob
import time
from typing import Optional, Tuple, Dict, List

try:
    import serial
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

# ANSI colors
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
CYAN = '\033[0;36m'
NC = '\033[0m'


def print_info(msg):
    print(f"{BLUE}[i]{NC} {msg}")


def print_ok(msg):
    print(f"{GREEN}[✓]{NC} {msg}")


def print_warn(msg):
    print(f"{YELLOW}[!]{NC} {msg}")


def print_error(msg):
    print(f"{RED}[✗]{NC} {msg}")


def print_step(msg):
    print(f"{CYAN}[→]{NC} {msg}")


def get_ttyusb_devices() -> List[str]:
    """Get list of all /dev/ttyUSB* devices."""
    return sorted(glob.glob('/dev/ttyUSB*'))


def probe_esp32(port: str, timeout: float = 1.5) -> bool:
    """Check if device is ESP32 by detecting 'PKT,' telemetry stream or 'PONG_ESP32'."""
    if not HAS_PYSERIAL:
        return False
    try:
        ser = serial.Serial(port, 115200, timeout=timeout)
        time.sleep(0.1)
        ser.reset_input_buffer()

        # Send PING just in case firmware supports it
        try:
            ser.write(b'PING\n')
            ser.flush()
        except Exception:
            pass

        start = time.time()
        response = b''
        while time.time() - start < timeout:
            data = ser.read(1024)
            if data:
                response += data
                if b'PKT,' in response or b'PONG_ESP32' in response:
                    ser.close()
                    return True
            else:
                time.sleep(0.05)

        ser.close()
        return b'PKT,' in response or b'PONG_ESP32' in response
    except Exception:
        return False


def check_rplidar_stream(port: str, timeout: float = 1.5) -> bool:
    """Check if device is RPLiDAR by toggling DTR line across common baud rates (115200, 256000)."""
    if not HAS_PYSERIAL:
        return False

    for baud in [115200, 256000]:
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
            
            # Enable DTR signal to trigger LiDAR motor rotation
            ser.dtr = True
            ser.rts = False
            time.sleep(0.2)
            ser.reset_input_buffer()

            data = ser.read(2048)
            ser.close()

            if not data:
                continue

            # Ensure data is not ESP32 text and contains LiDAR sync byte 0xA5
            if b'PKT,' not in data and (b'\xa5' in data or b'\x5a\xa5' in data):
                return True
        except Exception:
            pass

    return False


def probe_device(port: str) -> Tuple[Optional[str], float]:
    """Probe a single port to identify device type."""
    print_step(f"Probing {port}...")

    print_info("  Checking for ESP32 (Telemetry/PING)...")
    if probe_esp32(port):
        return ('esp32', 1.0)

    print_info("  Checking for RPLiDAR (Data stream/DTR)...")
    if check_rplidar_stream(port):
        return ('rplidar', 0.9)

    print_warn(f"  Unknown device on {port}")
    return (None, 0.0)


def create_symlink(target: str, link_name: str) -> bool:
    """Create symlink with proper permissions."""
    link_path = f'/dev/{link_name}'

    try:
        if os.path.exists(link_path) or os.path.islink(link_path):
            os.unlink(link_path)
    except PermissionError:
        print_error(f"Cannot remove {link_path} - need sudo")
        return False

    try:
        os.symlink(target, link_path)
        os.chmod(link_path, 0o666)
        os.chmod(target, 0o666)
        print_ok(f"Created: /dev/{link_name} -> {target}")
        return True
    except PermissionError:
        print_error(f"Cannot create symlink - need sudo")
        return False
    except Exception as e:
        print_error(f"Error creating symlink: {e}")
        return False


def auto_detect_and_link() -> bool:
    """Main detection logic."""
    print_info("Scanning for CP2102 USB-Serial devices...\n")

    devices = get_ttyusb_devices()
    if not devices:
        print_warn("No /dev/ttyUSB* devices found!")
        return False

    print_info(f"Found {len(devices)} device(s):")
    for d in devices:
        print(f"  - {d}")
    print()

    results: Dict[str, Tuple[Optional[str], float]] = {}
    for port in devices:
        dtype, conf = probe_device(port)
        results[port] = (dtype, conf)
        print()

    esp32_port = None
    rplidar_port = None

    for port, (dtype, _) in results.items():
        if dtype == 'esp32' and not esp32_port:
            esp32_port = port
        elif dtype == 'rplidar' and not rplidar_port:
            rplidar_port = port

    unknowns = [p for p, (t, _) in results.items() if t is None]

    # Fallback assignment
    if not esp32_port and not rplidar_port:
        if len(devices) >= 2:
            esp32_port = devices[0]
            rplidar_port = devices[1]
            print_warn("No positive identification. Using port order heuristic.")
    elif not esp32_port and unknowns:
        esp32_port = unknowns[0]
    elif not rplidar_port and unknowns:
        rplidar_port = unknowns[0]

    print_info("Creating symlinks...\n")
    success = True

    if esp32_port:
        success &= create_symlink(esp32_port, 'esp32')
    else:
        print_warn("ESP32 not detected - no /dev/esp32 symlink created")

    if rplidar_port:
        success &= create_symlink(rplidar_port, 'rplidar')
    else:
        print_warn("RPLiDAR not detected - no /dev/rplidar symlink created")

    print()
    print_info("Detection Summary:")
    print("=" * 40)
    for port, (dtype, conf) in results.items():
        conf_str = f"{conf:.0%}" if conf > 0 else "unknown"
        dev_name = dtype.upper() if dtype else "Unknown"
        print(f"  {port} -> {dev_name} ({conf_str})")
    print("=" * 40 + "\n")

    return success


def main():
    print("\n" + "=" * 50)
    print("  AMR USB Auto-Detection Script (Fixed)")
    print("=" * 50 + "\n")

    if os.geteuid() != 0:
        print_warn("Not running as root. Run with sudo for full functionality.")

    success = auto_detect_and_link()

    if success:
        print_ok("Auto-detection completed successfully!")
    else:
        print_error("Auto-detection failed or incomplete.")

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())