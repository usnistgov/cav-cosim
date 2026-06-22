"""
Query CARLA for all stop-sign world positions and print the ones near the
parked-bus scenario area (x ≈ -275, y ≈ -12 to -18).
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from carla_setup import setup_carla
setup_carla("0.9.16")
import carla

SCENARIO_X = -275.0
SCENARIO_Y = -15.0
RADIUS = 60.0  # metres around the scenario centre

client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
world = client.get_world()
current_map = world.get_map().name
print(f"Connected. Current map: {current_map}")

if "Town05_Opt" not in current_map:
    print("Loading Town05_Opt (the scenario map)...")
    world = client.load_world("Town05_Opt")
    print(f"Now on: {world.get_map().name}")

stop_actors = list(world.get_actors().filter("traffic.stop"))
print(f"\nFound {len(stop_actors)} stop-sign actors.\n")

candidates = []
for a in stop_actors:
    loc = a.get_location()
    dx = loc.x - SCENARIO_X
    dy = loc.y - SCENARIO_Y
    dist = (dx * dx + dy * dy) ** 0.5
    candidates.append((dist, a.id, loc, a.get_transform().rotation))

candidates.sort()

print(f"Stop signs within {RADIUS:.0f} m of scenario centre "
      f"(x={SCENARIO_X}, y={SCENARIO_Y}):\n")
print(f"{'dist (m)':>9}  {'id':>6}  "
      f"{'x':>8}  {'y':>8}  {'z':>6}  {'yaw':>7}")
print("-" * 60)
for dist, aid, loc, rot in candidates:
    if dist > RADIUS:
        break
    print(f"{dist:>9.2f}  {aid:>6}  "
          f"{loc.x:>8.2f}  {loc.y:>8.2f}  {loc.z:>6.2f}  "
          f"{rot.yaw:>7.2f}")

if not candidates or candidates[0][0] > RADIUS:
    print(f"\n(no stop signs within {RADIUS:.0f} m; "
          f"closest is {candidates[0][0]:.1f} m away)")
