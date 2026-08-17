"""End-to-end test for the chat assistant HTTP API (api_state/api_action/api_motion).

Runs web_controller with a fake serial port that injects telemetry frames and
captures written frames, on a throwaway port. No hardware needed.

Run:  py test_chat_api.py
"""

from __future__ import annotations

import asyncio
import json
import struct
import threading
import time
import unittest

import aiohttp

import web_controller
from protocol import FrameParser, MSG_CONTROL, MSG_FACE_EVENT, MSG_TELEMETRY, encode_frame

TEST_PORT = 8091
BASE = f"http://127.0.0.1:{TEST_PORT}"


def telemetry_frame(last_ctrl: int = 41) -> bytes:
    p = bytearray(85)
    p[0] = 3  # ARMED
    struct.pack_into("<H", p, 3, last_ctrl)   # last_control_sequence
    struct.pack_into("<H", p, 5, 120)         # link_age_ms
    p[7] = 0x0F                                # all 4 servos online
    p[9] = 1                                   # homed
    struct.pack_into("<i", p, 11, 1000)        # lift_position
    struct.pack_into("<i", p, 15, 1000)        # lift_target
    for i in range(4):
        b = 31 + i * 12
        p[b] = 1                               # online
        p[b + 3] = 35                          # temperature_c
        p[b + 4] = 70                          # temperature_limit_c
        p[b + 5] = 124                         # 12.4 V
        struct.pack_into("<h", p, b + 6, 2048)  # position
    return encode_frame(MSG_TELEMETRY, 1, bytes(p))


class FakeSerial:
    instances: list["FakeSerial"] = []

    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.written = bytearray()
        self._next_telemetry = 0.0
        FakeSerial.instances.append(self)

    def read(self, _size: int) -> bytes:
        now = time.monotonic()
        if now >= self._next_telemetry:
            self._next_telemetry = now + 0.2
            return telemetry_frame()
        time.sleep(0.02)
        return b""

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def close(self) -> None:
        self.is_open = False

    def control_payloads(self, since: int = 0) -> list[tuple[int, bytes]]:
        """Decode captured CONTROL/FACE_EVENT frames (optionally only bytes >= since)."""
        parser = FrameParser()
        payloads = []
        for frame in parser.feed(bytes(self.written[since:])):
            if frame.message_type in (MSG_CONTROL, MSG_FACE_EVENT):
                payloads.append((frame.message_type, frame.payload))
        return payloads


class ChatApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        web_controller.serial.Serial = FakeSerial
        web_controller.PORT = TEST_PORT
        web_controller.save_config = lambda _config: None  # keep the real config file clean
        cls.thread = threading.Thread(target=web_controller.main, daemon=True)
        cls.thread.start()
        for _ in range(50):
            try:
                import urllib.request
                urllib.request.urlopen(BASE + "/", timeout=0.5)
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError("server did not start")
        cls.backend = cls._find_backend()
        error = cls.backend.connect("FAKE")
        assert error is None, error

    @classmethod
    def tearDownClass(cls) -> None:
        asyncio.run_coroutine_threadsafe(
            cls.backend.disconnect(), cls.backend.loop
        ).result(timeout=3)

    @classmethod
    def _find_backend(cls):
        # main() stores it only in app["backend"]; grab via the class instances
        # is fragile, so capture through a wrapper set in setUpClass instead.
        return ChatApiTest._backend_ref

    def setUp(self) -> None:
        self.fake = FakeSerial.instances[-1]

    def test_01_state_api(self) -> None:
        time.sleep(0.5)  # let a few telemetry frames in
        status, data = http("GET", "/api/state")
        self.assertEqual(status, 200)
        self.assertTrue(data["connected"])
        self.assertTrue(data["sequence_synced"])
        self.assertTrue(data["telemetry_fresh"])
        self.assertEqual(data["telemetry"]["state"], 3)
        self.assertEqual(len(data["telemetry"]["servos"]), 4)

    def test_02_motion_pulse_and_expiry(self) -> None:
        scale = float(self.backend.config.get("speed_scale", 0.2))
        expected_vx = int(round(1.0 * scale * 1000.0))
        before = len(self.fake.written)
        status, data = http("POST", "/api/motion", {"vx": 1, "duration_ms": 800})
        self.assertEqual((status, data.get("ok")), (200, True))
        time.sleep(0.4)
        mid = decode_vx(self.fake.control_payloads(since=before))
        self.assertTrue(mid, "no CONTROL frames captured during pulse")
        self.assertIn(expected_vx, mid, f"pulse never reached {expected_vx}: {mid}")
        self.assertTrue(all(vx in (0, expected_vx) for vx in mid), f"pulse vx values: {mid}")
        time.sleep(0.6)  # total 1.0 s after POST: the 800 ms pulse has expired
        after = len(self.fake.written)
        time.sleep(0.45)  # let ~2 more frames come in
        tail = decode_vx(self.fake.control_payloads(since=after))
        self.assertTrue(tail, "no CONTROL frames captured after pulse")
        self.assertTrue(all(vx == 0 for vx in tail), f"post-pulse vx values: {tail}")

    def test_03_face_event_via_api(self) -> None:
        before = len(self.fake.written)
        status, data = http("POST", "/api/action", {"name": "face_event", "code": 3})
        self.assertEqual((status, data.get("ok")), (200, True))
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            events = [p[1] for p in self.fake.control_payloads() if p[0] == MSG_FACE_EVENT]
            if events:
                self.assertEqual(events[-1], bytes([3]))
                return
            time.sleep(0.05)
        self.fail("FACE_EVENT frame was not written")

    def test_04_action_arm_and_stop(self) -> None:
        status, data = http("POST", "/api/action", {"name": "arm_toggle"})
        self.assertEqual((status, data.get("ok")), (200, True))
        status, data = http("POST", "/api/action", {"name": "stop"})
        self.assertEqual((status, data.get("ok")), (200, True))

    def test_05_motion_rejected_when_disconnected(self) -> None:
        asyncio.run_coroutine_threadsafe(self.backend.disconnect(), self.backend.loop).result(timeout=3)
        try:
            status, data = http("POST", "/api/motion", {"vx": 1, "duration_ms": 500})
            self.assertEqual(status, 409)
            self.assertFalse(data["ok"])
        finally:
            error = self.backend.connect("FAKE")
            self.assertIsNone(error)


def http(method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
    import urllib.request
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(BASE + path, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def decode_vx(frames: list[tuple[int, bytes]]) -> list[int]:
    return [struct.unpack_from("<h", payload, 0)[0] for mt, payload in frames if mt == MSG_CONTROL]


# capture the backend instance created inside main()
_orig_init = web_controller.RobotBackend.__init__


def _capture_init(self):
    _orig_init(self)
    ChatApiTest._backend_ref = self


web_controller.RobotBackend.__init__ = _capture_init

if __name__ == "__main__":
    unittest.main(verbosity=2)
