#!/usr/bin/env python3
"""Quick diagnostic: prints raw WMET values with no conversion."""
from wemt_api import WemtAPI
import math

api = WemtAPI(device_ip="10.42.0.144")
try:
    api.start_tracking()
    print("Collecting 5 samples...\n")
    for i in range(5):
        pose = api.wait_for_pose("module-1", timeout_s=2.0)
        p = pose.position_mm
        q = pose.quaternion_wxyz
        dist = math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
        qnorm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        print(f"Sample {i+1}:")
        print(f"  position_mm : x={p[0]:.2f}  y={p[1]:.2f}  z={p[2]:.2f}")
        print(f"  distance_mm : {dist:.2f} mm  ({dist/10:.1f} cm)")
        print(f"  quaternion  : w={q[0]:.4f}  x={q[1]:.4f}  y={q[2]:.4f}  z={q[3]:.4f}")
        print(f"  quat norm   : {qnorm:.4f}  (should be 1.0)")
        print()
finally:
    api.stop_tracking()
