from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
import queue
import random
import sys
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WEB_DIR = ROOT / "web"
DEPS_DIR = ROOT / ".deps"
TWIST_DATA_DIR = ROOT.parent / "twist_bench" / "data"
IMU_IDS = ("IMU0", "IMU1", "IMU2", "IMU3", "IMU4")
POSITIONS = ("POSITION_0_TOP", "POSITION_1_MID", "POSITION_2_LOW", "POSITION_3_EXTRA", "POSITION_4_EXTRA")
ADDRS = ("0x32", "0x33", "0x34", "0x35", "0x36")
if DEPS_DIR.exists():
    sys.path.insert(0, str(DEPS_DIR))


class Recorder:
    def __init__(self, data_dir: Path) -> None:
        self.data_dir = data_dir
        self._lock = threading.Lock()
        self._fh = None
        self.active = False
        self.line_count = 0
        self.started_at: float | None = None
        self.stopped_at: float | None = None
        self.log_path: Path | None = None
        self.marker_path: Path | None = None
        self.metadata: dict[str, object] = {}

    def start(self, metadata: dict[str, object]) -> dict[str, object]:
        with self._lock:
            if self.active:
                raise RuntimeError("Recording is already active")
            self.data_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            stem = f"visual_test_{stamp}"
            self.log_path = self.data_dir / f"{stem}.log"
            self.marker_path = self.data_dir / f"{stem}_markers.json"
            self._fh = self.log_path.open("w", encoding="utf-8", newline="\n")
            self.active = True
            self.line_count = 0
            self.started_at = time.time()
            self.stopped_at = None
            self.metadata = {
                "protocol": metadata.get("protocol", "pair_smoke_test"),
                "parent": metadata.get("parent", "IMU1"),
                "child": metadata.get("child", "IMU2"),
                "phases": metadata.get("phases", []),
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "log_path": str(self.log_path),
                "marker_path": str(self.marker_path),
                "notes": metadata.get("notes", ""),
            }
            self._write_markers_locked(active=True)
            return self.status()

    def stop(self, reason: str = "completed") -> dict[str, object]:
        with self._lock:
            if not self.active:
                return self.status()
            self.active = False
            self.stopped_at = time.time()
            if self._fh:
                self._fh.flush()
                self._fh.close()
                self._fh = None
            self.metadata["stop_reason"] = reason
            self._write_markers_locked(active=False)
            return self.status()

    def write_line(self, line: str) -> None:
        with self._lock:
            if not self.active or self._fh is None:
                return
            self._fh.write(line + "\n")
            self.line_count += 1

    def status(self) -> dict[str, object]:
        elapsed = 0.0
        if self.started_at is not None:
            end = time.time() if self.active else (self.stopped_at or time.time())
            elapsed = max(0.0, end - self.started_at)
        return {
            "active": self.active,
            "line_count": self.line_count,
            "elapsed_s": elapsed,
            "log_path": str(self.log_path) if self.log_path else None,
            "marker_path": str(self.marker_path) if self.marker_path else None,
            "metadata": self.metadata,
        }

    def _write_markers_locked(self, active: bool) -> None:
        if not self.marker_path:
            return
        payload = {
            **self.metadata,
            "active": active,
            "line_count": self.line_count,
            "started_server_time": self.started_at,
            "stopped_server_time": self.stopped_at,
        }
        self.marker_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


class Hub:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subscribers: list[queue.Queue[str]] = []
        self.latest: dict[str, dict] = {}
        self.status = {
            "type": "status",
            "serial": "starting",
            "message": "Bridge starting",
            "server_time": time.time(),
        }

    def subscribe(self) -> queue.Queue[str]:
        q: queue.Queue[str] = queue.Queue(maxsize=100)
        with self._lock:
            self._subscribers.append(q)
            self._push_to(q, self.status)
            for sample in self.latest.values():
                self._push_to(q, sample)
        return q

    def unsubscribe(self, q: queue.Queue[str]) -> None:
        with self._lock:
            if q in self._subscribers:
                self._subscribers.remove(q)

    def publish(self, payload: dict) -> None:
        payload["server_time"] = time.time()
        if payload.get("type") == "sample":
            self.latest[payload["imu"]] = payload
        elif payload.get("type") == "status":
            self.status = payload

        text = json.dumps(payload, separators=(",", ":"))
        with self._lock:
            for q in list(self._subscribers):
                self._push_to(q, text)

    def _push_to(self, q: queue.Queue[str], payload: dict | str) -> None:
        text = payload if isinstance(payload, str) else json.dumps(payload, separators=(",", ":"))
        try:
            q.put_nowait(text)
        except queue.Full:
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            try:
                q.put_nowait(text)
            except queue.Full:
                pass


def parse_sample(line: str) -> dict | None:
    parts = line.strip().split()
    if len(parts) != 10 or not parts[1].startswith("IMU"):
        return None

    try:
        t_ms = int(parts[0])
        imu = parts[1]
        position = parts[2]
        addr = parts[3]
        ax = float(parts[4])
        ay = float(parts[5])
        az = float(parts[6])
        gx = float(parts[7])
        gy = float(parts[8])
        gz = float(parts[9])
    except ValueError:
        return None

    g_mag = math.sqrt(ax * ax + ay * ay + az * az)
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

    return {
        "type": "sample",
        "t_ms": t_ms,
        "imu": imu,
        "position": position,
        "addr": addr,
        "ax_mg": ax,
        "ay_mg": ay,
        "az_mg": az,
        "gx_dps": gx,
        "gy_dps": gy,
        "gz_dps": gz,
        "g_mag": g_mag,
        "roll_deg": roll,
        "pitch_deg": pitch,
    }


class SerialWorker(threading.Thread):
    def __init__(self, hub: Hub, recorder: Recorder, port: str, baud: int) -> None:
        super().__init__(daemon=True)
        self.hub = hub
        self.recorder = recorder
        self.requested = port
        self.baud = baud

    @staticmethod
    def _resolve_port(list_ports, requested: str) -> str | None:
        """Pick a serial port. An explicit COM name is used as-is; "auto" enumerates ports and
        prefers an ST-Link / STM32 virtual COM port (the Nucleo onboard debugger), so the viewer
        keeps working after replugging USB or moving to another machine where the COM number changed."""
        if requested and requested.lower() != "auto":
            return requested
        ports = list(list_ports.comports())
        for p in ports:
            haystack = " ".join(
                filter(None, [p.description, p.manufacturer, getattr(p, "product", None)])
            ).lower()
            if "stlink" in haystack or "st-link" in haystack or "stmicro" in haystack or p.vid == 0x0483:
                return p.device
        return ports[0].device if ports else None

    def run(self) -> None:
        try:
            import serial
            from serial.tools import list_ports
        except ModuleNotFoundError:
            self.hub.publish(
                {
                    "type": "status",
                    "serial": "error",
                    "message": "pyserial not found. Run pip install pyserial.",
                }
            )
            return

        while True:
            port = self._resolve_port(list_ports, self.requested)
            if port is None:
                self.hub.publish(
                    {
                        "type": "status",
                        "serial": "error",
                        "message": "No serial port found (looked for an ST-Link / STM32 VCP). "
                        "Plug in the Nucleo, or pass --serial COMx.",
                    }
                )
                time.sleep(2.0)
                continue
            try:
                self.hub.publish(
                    {
                        "type": "status",
                        "serial": "connecting",
                        "message": f"Opening {port} @ {self.baud}",
                    }
                )
                with serial.Serial(port, self.baud, timeout=1) as ser:
                    ser.reset_input_buffer()
                    self.hub.publish(
                        {
                            "type": "status",
                            "serial": "open",
                            "message": f"{port} open",
                        }
                    )
                    for raw in ser:
                        line = raw.decode("utf-8", errors="ignore").strip()
                        if line:
                            self.recorder.write_line(line)
                        sample = parse_sample(line)
                        if sample:
                            self.hub.publish(sample)
            except Exception as exc:
                self.hub.publish(
                    {
                        "type": "status",
                        "serial": "error",
                        "message": f"{port}: {exc}",
                    }
                )
                time.sleep(1.5)


class DemoWorker(threading.Thread):
    def __init__(self, hub: Hub) -> None:
        super().__init__(daemon=True)
        self.hub = hub

    def run(self) -> None:
        self.hub.publish({"type": "status", "serial": "demo", "message": "Demo data active"})
        t0 = time.time()
        while True:
            now = time.time() - t0
            for idx, imu in enumerate(IMU_IDS):
                roll = math.sin(now * (0.9 + idx * 0.17) + idx) * 42
                pitch = math.cos(now * (0.7 + idx * 0.11) + idx * 0.8) * 30
                sample = sample_from_angles(imu, idx, int(now * 1000), roll, pitch)
                self.hub.publish(sample)
            time.sleep(0.05)


def sample_from_angles(imu: str, idx: int, t_ms: int, roll_deg: float, pitch_deg: float) -> dict:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    ax = -math.sin(pitch) * 1000
    ay = math.sin(roll) * math.cos(pitch) * 1000
    az = math.cos(roll) * math.cos(pitch) * 1000
    jitter = lambda: random.uniform(-3.0, 3.0)
    return {
        "type": "sample",
        "t_ms": t_ms,
        "imu": imu,
        "position": POSITIONS[idx],
        "addr": ADDRS[idx],
        "ax_mg": ax + jitter(),
        "ay_mg": ay + jitter(),
        "az_mg": az + jitter(),
        "gx_dps": random.uniform(-1.0, 1.0),
        "gy_dps": random.uniform(-1.0, 1.0),
        "gz_dps": random.uniform(-1.0, 1.0),
        "g_mag": 1000.0,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
    }


class ViewerHandler(SimpleHTTPRequestHandler):
    hub: Hub
    recorder: Recorder

    def do_GET(self) -> None:
        if self.path == "/events":
            self.handle_events()
            return
        if self.path == "/api/latest":
            body = json.dumps({"status": self.hub.status, "latest": self.hub.latest}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/record/status":
            self.send_json(200, self.recorder.status())
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path == "/api/record/start":
            body = self.read_json_body()
            try:
                status = self.recorder.start(body)
            except RuntimeError as exc:
                self.send_json(409, {"error": str(exc), "recording": self.recorder.status()})
                return
            self.hub.publish(
                {
                    "type": "status",
                    "serial": self.hub.status.get("serial", "open"),
                    "message": f"Recording: {Path(status['log_path']).name if status.get('log_path') else 'active'}",
                }
            )
            self.send_json(200, status)
            return
        if self.path == "/api/record/stop":
            body = self.read_json_body()
            reason = str(body.get("reason", "completed"))
            status = self.recorder.stop(reason=reason)
            self.hub.publish(
                {
                    "type": "status",
                    "serial": self.hub.status.get("serial", "open"),
                    "message": f"Recording stopped ({status['line_count']} lines)",
                }
            )
            self.send_json(200, status)
            return
        self.send_error(404)

    def read_json_body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}

    def send_json(self, code: int, payload: dict) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def handle_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        q = self.hub.subscribe()
        try:
            while True:
                try:
                    payload = q.get(timeout=15)
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.hub.unsubscribe(q)

    def log_message(self, fmt: str, *args) -> None:
        if self.path != "/events":
            super().log_message(fmt, *args)


def main() -> None:
    parser = argparse.ArgumentParser(description="5-IMU cube viewer serial bridge")
    parser.add_argument("--serial", default="auto", help="Serial port (e.g. COM3), or 'auto' to pick the ST-Link/STM32 VCP. Default auto.")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port")
    parser.add_argument("--demo", action="store_true", help="Run without serial hardware")
    args = parser.parse_args()

    if not WEB_DIR.exists():
        raise SystemExit(f"Missing web directory: {WEB_DIR}")

    hub = Hub()
    recorder = Recorder(TWIST_DATA_DIR)
    ViewerHandler.hub = hub
    ViewerHandler.recorder = recorder
    handler = partial(ViewerHandler, directory=str(WEB_DIR))
    server = ThreadingHTTPServer((args.host, args.port), handler)

    worker: threading.Thread
    if args.demo:
        worker = DemoWorker(hub)
    else:
        worker = SerialWorker(hub, recorder, args.serial, args.baud)
    worker.start()

    url = f"http://{args.host}:{args.port}"
    print(f"IMU cube viewer: {url}")
    print("Stop with Ctrl+C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
