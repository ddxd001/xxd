#!/usr/bin/env python3
"""ra8p1.py — CLI for the RA8P1 omni robot via the local web controller backend.

Talks to the aiohttp backend of pc_controller/web_controller.py over HTTP
(default http://127.0.0.1:8080). The backend owns the LoRa serial link and all
frame sequencing/safety logic; this CLI is a thin wrapper.

Usage:
  ra8p1.py state                          # connection + full telemetry snapshot
  ra8p1.py move [--vx V] [--vy V] [--omega W] [--lift L] [--ms N]
                                          # timed motion pulse, components -1..1, ms <= 3000
  ra8p1.py arm                            # toggle arm/disarm (backend gates on telemetry)
  ra8p1.py stop                           # immediate disarm (always safe)
  ra8p1.py clear_fault                    # attempt fault clear
  ra8p1.py face <code|name>               # screen/speaker event 0..10 or Chinese name
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.request
import urllib.error

BASE = os.environ.get("RA8P1_API", "http://127.0.0.1:8080")

FACE_NAMES = {
    "待机": 0x00, "启动": 0x01, "欢迎回家": 0x02, "移动中": 0x03, "机器人移动中": 0x03,
    "请缓慢移动": 0x04, "缓慢移动": 0x04, "请停止": 0x05, "停止": 0x05,
    "开始充电": 0x06, "充电完成": 0x07, "检测到儿童": 0x08, "儿童": 0x08,
    "任务完成": 0x09, "安全停靠": 0x0A, "停靠": 0x0A,
}
FACE_CODES = {
    0x00: "待机", 0x01: "启动", 0x02: "欢迎回家", 0x03: "机器人移动中", 0x04: "请缓慢移动",
    0x05: "请停止", 0x06: "开始充电", 0x07: "充电完成", 0x08: "检测到儿童",
    0x09: "任务完成", 0x0A: "安全停靠",
}


def request(method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(BASE + path, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            return exc.code, json.loads(exc.read().decode("utf-8"))
        except ValueError:
            return exc.code, {"ok": False, "error": str(exc)}
    except OSError as exc:
        print(f"无法连接上位机后端 {BASE}（web_controller.py 是否在运行？）：{exc}")
        sys.exit(2)


def print_state() -> None:
    _, data = request("GET", "/api/state")
    print(f"连接: {data.get('text')}  端口: {data.get('port')}  序号同步: {data.get('sequence_synced')}")
    tel = data.get("telemetry")
    if not tel:
        print("遥测: 无（未收到或已断开）")
        return
    fresh = "新鲜" if data.get("telemetry_fresh") else "过旧(>1s)"
    print(f"遥测: {fresh} | 状态: {tel['state_name']} | 故障: {tel['fault_text']}")
    print(f"链路: {tel['link_age_ms']} ms | MCU已收序号: {tel['last_control_sequence']}")
    print(f"升降: 位置 {tel['lift_position']} / 目标 {tel['lift_target']} | 上电零点有效: {tel['homed']}")
    print(f"轮速目标: {list(tel['wheel_speed'])}")
    for i, s in enumerate(tel["servos"], 1):
        name = f"ID{i}" + ("(升降)" if i == 4 else "")
        print(f"  {name}: {'在线' if s['online'] else '掉线'} {s['voltage_v']}V "
              f"{s['temperature_c']}°C 位{s['position']} 速{s['speed']} 流{s['current']} {s['fault_text']}")
    snap = tel.get("fault_snapshot") or {}
    if snap.get("valid"):
        print(f"故障快照: ID{snap['servo_id']} E=0x{snap['protocol_error']:02X} "
              f"S=0x{snap['status_flags']:02X} {snap['temperature_c']}°C {snap['fault_text']}")


def main() -> None:
    parser = argparse.ArgumentParser(description="RA8P1 omni robot CLI (via web controller backend)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("state", help="print connection + telemetry snapshot")

    p_move = sub.add_parser("move", help="timed motion pulse")
    p_move.add_argument("--vx", type=float, default=0.0, help="forward +, -1..1")
    p_move.add_argument("--vy", type=float, default=0.0, help="left +, -1..1")
    p_move.add_argument("--omega", type=float, default=0.0, help="CCW +, -1..1")
    p_move.add_argument("--lift", type=float, default=0.0, help="up +, -1..1")
    p_move.add_argument("--ms", type=int, default=1000, help="duration, 100..3000 ms")

    sub.add_parser("arm", help="toggle arm/disarm")
    sub.add_parser("stop", help="immediate disarm")
    sub.add_parser("clear_fault", help="attempt fault clear")

    p_face = sub.add_parser("face", help="screen/speaker event")
    p_face.add_argument("event", help="0..10 or Chinese name (欢迎回家/请停止/安全停靠/...)")

    args = parser.parse_args()

    if args.cmd == "state":
        print_state()
    elif args.cmd == "move":
        status, data = request("POST", "/api/motion", {
            "vx": args.vx, "vy": args.vy, "omega": args.omega, "lift": args.lift,
            "duration_ms": args.ms,
        })
        print(json.dumps(data, ensure_ascii=False))
        sys.exit(0 if status == 200 and data.get("ok") else 1)
    elif args.cmd in ("arm", "stop", "clear_fault"):
        name = {"arm": "arm_toggle", "stop": "stop", "clear_fault": "clear_fault"}[args.cmd]
        status, data = request("POST", "/api/action", {"name": name})
        print(json.dumps(data, ensure_ascii=False))
        sys.exit(0 if status == 200 and data.get("ok") else 1)
    elif args.cmd == "face":
        event = args.event.strip()
        if event in FACE_NAMES:
            code = FACE_NAMES[event]
        else:
            try:
                code = int(event, 0)
            except ValueError:
                print(f"未知事件：{event}。可用名称：{'、'.join(sorted(set(FACE_CODES.values())))} 或 0~10")
                sys.exit(2)
        status, data = request("POST", "/api/action", {"name": "face_event", "code": code})
        if data.get("ok"):
            print(f"已排队发送事件 0x{code:02X}（{FACE_CODES.get(code, '?')}）；Linux 端无执行确认")
        else:
            print(json.dumps(data, ensure_ascii=False))
        sys.exit(0 if status == 200 and data.get("ok") else 1)


if __name__ == "__main__":
    main()
