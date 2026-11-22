#!/usr/bin/env python3
"""
ELE v9.0.0 — Pure-Stdlib M5Stack Cardputer-Adv Hardware Data Capture Tool
=========================================================================
Connects over USB CDC serial (115200 baud via stdlib termios/select) and
captures real physical measurements into `data/cardputer/`:
  1. `cardputer_rf_csi.csv`     (2.4 GHz live OFDM Wi-Fi CSI complex I/Q frames)
  2. `cardputer_thermal_rail.csv` (ESP32-S3 die self-heating/cooling + rail droop)
  3. `cardputer_gpio_rc_adc.csv`  (12-bit factory ADC noise, GPIO1 step & I2C scan)
  4. `hardware_manifest.json`     (anonymized board/chip ID, timestamps & SHA-256)
"""
import argparse
import datetime
import hashlib
import json
import os
import select
import sys
import termios
import time


class CdcSerial:
    def __init__(self, port, baud=termios.B115200):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)) | termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = baud
        attrs[5] = baud
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._buf = bytearray()

    def close(self):
        os.close(self.fd)

    def send_cmd(self, cmd):
        data = (cmd.strip() + "\n").encode("ascii")
        os.write(self.fd, data)

    def read_line(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            nl = self._buf.find(b"\n")
            if nl >= 0:
                line = bytes(self._buf[:nl]).decode("utf-8", errors="replace").strip("\r\n ")
                del self._buf[: nl + 1]
                return line
            r, _, _ = select.select([self.fd], [], [], 0.1)
            if r:
                chunk = os.read(self.fd, 8192)
                if chunk:
                    self._buf.extend(chunk)
        return None


def wait_for_pong(ser):
    for _ in range(10):
        ser.send_cmd("PING")
        t0 = time.time()
        while time.time() - t0 < 1.5:
            ln = ser.read_line(timeout=0.5)
            if ln and ln.startswith("PONG,"):
                return ln
        time.sleep(0.3)
    raise RuntimeError("Timed out waiting for PONG from Cardputer-Adv firmware")


def capture_until(ser, cmd, end_marker, timeout=30.0):
    ser.send_cmd(cmd)
    lines = []
    t0 = time.time()
    while time.time() - t0 < timeout:
        ln = ser.read_line(timeout=1.0)
        if ln is None:
            continue
        if ln == end_marker:
            return lines
        lines.append(ln)
    raise RuntimeError(f"Timed out waiting for {end_marker} after {cmd} (got {len(lines)} lines)")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(65536)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description="Capture real Cardputer-Adv measurements for ELE")
    ap.add_argument("--port", default="/dev/cu.usbmodem101")
    ap.add_argument("--out-dir", default="data/cardputer")
    ap.add_argument("--power-only", action="store_true",
                    help="Capture only EXP_POWER_MODES into cardputer_power_modes.csv and preserve existing CSVs")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    ser = CdcSerial(args.port)
    scan_lines, csi_lines, therm_lines, adc_lines, rc_lines, i2c_lines = [], [], [], [], [], []
    power_lines = []
    try:
        pong = wait_for_pong(ser)
        print(f"  [CARDPUTER] Connected: {pong}")
        pong_kv = {}
        for part in pong.split(",")[2:]:
            if "=" in part:
                k, v = part.split("=", 1)
                pong_kv[k] = v

        if not args.power_only:
            csi_raw = capture_until(ser, "EXP_CSI", "END_CSI", timeout=20.0)
            scan_lines = [l for l in csi_raw if l.startswith("SCAN,")]
            csi_lines = [l for l in csi_raw if l.startswith("CSI,")]
            print(f"  [EXP_CSI] AP channels seen={len(scan_lines)}, real CSI frames={len(csi_lines)}")
            if not csi_lines:
                raise RuntimeError("No real CSI frames captured from 2.4 GHz radio!")

            therm_raw = capture_until(ser, "EXP_THERMAL", "END_THERMAL", timeout=35.0)
            therm_lines = [l for l in therm_raw if l.startswith("THERM,")]
            print(f"  [EXP_THERMAL] Captured {len(therm_lines)} real thermal/rail samples")
            if len(therm_lines) < 100:
                raise RuntimeError(f"Expected >=100 thermal samples, got {len(therm_lines)}")

            gpio_raw = capture_until(ser, "EXP_GPIO_RC", "END_GPIO_RC", timeout=10.0)
            adc_lines = [l for l in gpio_raw if l.startswith("ADC,")]
            rc_lines = [l for l in gpio_raw if l.startswith("RC,")]
            i2c_lines = [l for l in gpio_raw if l.startswith("I2C,")]
            print(f"  [EXP_GPIO_RC] ADC={len(adc_lines)}, RC={len(rc_lines)}, GroveI2C={len(i2c_lines)}")

        pwr_raw = capture_until(ser, "EXP_POWER_MODES", "END_POWER_MODES", timeout=25.0)
        power_lines = [l for l in pwr_raw if l.startswith("PWR,")]
        print(f"  [EXP_POWER_MODES] Captured {len(power_lines)} real power-mode/PDN samples")
        if len(power_lines) < 200:
            raise RuntimeError(f"Expected >=200 power-mode samples, got {len(power_lines)}")
    finally:
        ser.close()

    if not args.power_only:
        csi_path = os.path.join(args.out_dir, "cardputer_rf_csi.csv")
        with open(csi_path, "w") as f:
            f.write(f"# board=M5Stack-Cardputer-Adv chip=ESP32-S3 {pong}\n")
            for s in scan_lines:
                f.write(f"# {s}\n")
            f.write("seq,t_us,ch,rssi,noise_floor,len,iq_pairs\n")
            for l in csi_lines:
                f.write(l[4:] + "\n")

        therm_path = os.path.join(args.out_dir, "cardputer_thermal_rail.csv")
        with open(therm_path, "w") as f:
            f.write(f"# board=M5Stack-Cardputer-Adv chip=ESP32-S3 {pong}\n")
            f.write("idx,t_s,v_rail_mv,t_die_c,load_on\n")
            for l in therm_lines:
                f.write(l[6:] + "\n")

        gpio_path = os.path.join(args.out_dir, "cardputer_gpio_rc_adc.csv")
        with open(gpio_path, "w") as f:
            f.write(f"# board=M5Stack-Cardputer-Adv chip=ESP32-S3 {pong}\n")
            for il in i2c_lines:
                f.write(f"# {il}\n")
            f.write("kind,idx,t_us,mv\n")
            for l in adc_lines + rc_lines:
                f.write(l + "\n")

    pwr_path = os.path.join(args.out_dir, "cardputer_power_modes.csv")
    with open(pwr_path, "w") as f:
        f.write(f"# board=M5Stack-Cardputer-Adv chip=ESP32-S3 {pong}\n")
        f.write("sample_idx,t_ms,mode,cpu_mhz,v_rail_mv,t_die_c\n")
        for l in power_lines:
            f.write(l[4:] + "\n")

    man_path = os.path.join(args.out_dir, "hardware_manifest.json")
    if args.power_only and os.path.exists(man_path):
        with open(man_path, "r") as f:
            manifest = json.load(f)
    else:
        manifest = {
            "board": "M5Stack Cardputer-Adv",
            "chip": "ESP32-S3",
            "rev": pong_kv.get("rev", "unknown"),
            "cpu_mhz": int(pong_kv.get("cpu_mhz", "240")),
            "mac_sha8": pong_kv.get("mac_sha8", ""),
            "counts": {},
            "files": {},
        }
    manifest["captured_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    if not args.power_only:
        manifest["counts"].update({
            "scan_aps": len(scan_lines),
            "csi_frames": len(csi_lines),
            "thermal_samples": len(therm_lines),
            "adc_samples": len(adc_lines),
            "rc_samples": len(rc_lines),
            "grove_i2c_devices": len(i2c_lines),
        })
        manifest["files"].update({
            "cardputer_rf_csi.csv": sha256_file(os.path.join(args.out_dir, "cardputer_rf_csi.csv")),
            "cardputer_thermal_rail.csv": sha256_file(os.path.join(args.out_dir, "cardputer_thermal_rail.csv")),
            "cardputer_gpio_rc_adc.csv": sha256_file(os.path.join(args.out_dir, "cardputer_gpio_rc_adc.csv")),
        })
    manifest["counts"]["power_samples"] = len(power_lines)
    manifest["files"]["cardputer_power_modes.csv"] = sha256_file(pwr_path)

    with open(man_path, "w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")
    print(f"  [SAVED] {args.out_dir} ({man_path})")


if __name__ == "__main__":
    main()
