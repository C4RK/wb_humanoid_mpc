#!/usr/bin/env python3
"""Read the G1 locomotion FSM without registering any setter API.

This utility sends getter RPC requests to the ``sport`` service and prints the
raw values returned by the robot.  It does not create a joint-command
publisher and cannot change the robot mode.
"""

import argparse
import json
import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    LOCO_API_VERSION,
    LOCO_SERVICE_NAME,
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)
from unitree_sdk2py.rpc.client import Client


FSM_LABELS = {
    0: "zero torque / 零力矩",
    1: "damping / 阻尼",
    2: "position squat / 位控下蹲",
    3: "position sit / 位控落座",
    4: "locked standing / 锁定站立（无自主平衡）",
    500: "regular motion control, 1-DoF waist / 常规运控（单腰）",
    501: "regular motion control, 3-DoF waist / 常规运控（三腰）",
    702: "lie-to-stand action / 躺起动作",
    706: "squat/stand action / 蹲起动作",
    801: "walk-run control / 走跑运控",
}


class ReadOnlyLocoClient(Client):
    """A minimal client that registers getter API IDs only."""

    def __init__(self):
        super().__init__(LOCO_SERVICE_NAME, False)

    def Init(self) -> None:
        self._SetApiVerson(LOCO_API_VERSION)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, 0)

    def get_raw(self, api_id: int):
        return self._Call(api_id, "{}")


def decode_integer(raw):
    """Decode the SDK service's {"data": integer} response conservatively."""
    if raw is None:
        return None
    try:
        payload = json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, json.JSONDecodeError):
        return None
    value = payload.get("data") if isinstance(payload, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(float(value)) or int(value) != value:
        return None
    return int(value)


def query(client: ReadOnlyLocoClient, api_id: int, name: str):
    code, raw = client.get_raw(api_id)
    value = decode_integer(raw) if code == 0 else None
    if code != 0:
        return f"{name}: RPC error={code}, raw={raw!r}", None
    if value is None:
        return f"{name}: undecodable response raw={raw!r}", None
    return f"{name}: {value}", value


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read G1 sport-service FSM values without registering setters."
    )
    parser.add_argument("interface", help="wired interface connected to the G1")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="seconds to monitor (default: 10)",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="seconds between queries (default: 1)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="RPC timeout in seconds (default: 2)",
    )
    args = parser.parse_args()
    if args.duration <= 0.0 or args.period <= 0.0 or args.timeout <= 0.0:
        parser.error("--duration, --period and --timeout must be positive")

    ChannelFactoryInitialize(0, args.interface)
    client = ReadOnlyLocoClient()
    client.SetTimeout(args.timeout)
    client.Init()

    print(
        f"Read-only G1 FSM monitor on {args.interface}; service={LOCO_SERVICE_NAME}."
    )
    print("No setter API and no joint-command publisher were registered.")
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        fsm_text, fsm_id = query(
            client, ROBOT_API_ID_LOCO_GET_FSM_ID, "fsm_id"
        )
        mode_text, _ = query(
            client, ROBOT_API_ID_LOCO_GET_FSM_MODE, "fsm_mode"
        )
        balance_text, _ = query(
            client, ROBOT_API_ID_LOCO_GET_BALANCE_MODE, "balance_mode"
        )
        label = FSM_LABELS.get(fsm_id, "unknown; keep the raw value")
        print(f"{fsm_text} ({label}); {mode_text}; {balance_text}")
        time.sleep(min(args.period, max(0.0, deadline - time.monotonic())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
