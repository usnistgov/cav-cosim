# Foxglove Dashboard

Visualization layer for the VRU AEB demo. Connects to ROS2 via the Foxglove
bridge running inside `osc2_engine`/`run_sweep.py`.

## Layout

```
dashboard/
├── layouts/
│   └── vru_demo.json          # canonical demo layout
└── extensions/
    └── horizontal-bar/        # custom throttle/brake bar panel (source)
```

Foxglove loads installed extensions from `~/.foxglove-studio/extensions/`,
not from this folder — see "Install / rebuild the custom panel" below.

## Open the demo

1. Start CARLA + the scenario via `run_sweep.py` (this spawns the AEB,
   perception, chase relay, and the Foxglove bridge on `ws://localhost:8765`).
2. Launch Foxglove Studio, connect to `ws://localhost:8765`.
3. `File → Import layout from file…` → `dashboard/layouts/vru_demo.json`.

## Install / rebuild the custom panel

The `horizontal-bar` panel is what drives the THROTTLE / BRAKE bars in the
layout. Whenever its source changes:

```bash
cd dashboard/extensions/horizontal-bar
npm install        # first time only
npm run local-install
```

`local-install` copies the built panel to
`~/.foxglove-studio/extensions/unknown.horizontal-bar-0.0.0/`. Restart
Foxglove Studio after installing to pick up changes.

## Topics consumed by the layout

| Panel | Topic |
|---|---|
| Chase view | `/dashboard/chase_image` (republished by `osc2_engine/tools/chase_relay.py`) |
| Perception view | `/perception/image_debug` |
| Scenario info | `/metrics/scenario_info` |
| Ego speed (gauge + text + plot) | `/metrics/ego_speed_kmh`, `/metrics/target_speed_kmh` |
| Ped speed | `/metrics/ped_speed` |
| Throttle / Brake bars | `/metrics/throttle`, `/metrics/brake` |
| State timeline | `/metrics/perception_state`, `/metrics/state` |
| Distance plot | `/metrics/ped_distance_plot` (30 m baseline when no detection) |

All metric publishers live in
`ros_workspace/src/carla_aeb_agent/carla_aeb_agent/aeb_node_yolo.py`.
