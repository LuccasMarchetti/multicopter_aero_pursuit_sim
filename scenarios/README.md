# Scenarios

A scenario is a YAML file describing one reproducible chase: the **target** (the pursued drone)
and the path it flies, the **pursuer** (PX4) start and how the chase begins, and how many runs.
Edit them in the GCS (**Scenarios** tab: form + map editor) or by hand; the `scenario_runner`
executes them, in Gazebo or Isaac Sim.

- Every field, commented: [`reference.yaml`](reference.yaml) (a valid scenario).
- Help in the GCS: Scenarios → **? Help** (fields, path kinds, CSV format, YAML reference).
- Validate / preview without a simulation: `ros2 run aero_pursuit scenario_tool preview <file>`.
- Run: GCS → Scenarios → ▶ Run, or `./ros2_ws/run_scenario.sh <file>`.
- Studies (scenarios × variants × repetitions): [`studies/`](studies/) and `run_study.py`.

Units: meters, seconds, degrees (`*_deg`); world ENU coordinates (x east, y north, z up).

## The target path

`target.path` is a list of legs flown in order (`target.loops` repeats the whole list; 0 = forever):

| Leg | What it does |
|---|---|
| `trajectory: {type: waypoints, points: [...]}` | Through the points at `speed` (constant; per point `speed`/`hold`), straight or `spline` |
| `trajectory: {type: timed, points: [[t, x, y, z], ...]}` | At each point at its time (from a CSV/Excel table): variable speed |
| `trajectory: {type: circle / polygon / figure8 / lissajous / helix / random, ...}` | A shape from a few parameters, at constant speed |
| `goto: [x, y, z]` | Straight line to a point |
| `hover: s` | Stays still |
| `behavior: flee / random / orbit` | Reacts to the pursuer instead of following a fixed path |

A leg can also name a trajectory of the `trajectories:` library (shared by several legs).

## Files

| File | What it exercises |
|---|---|
| `figure8_basic.yaml` | Figure 8 forever, vision, automatic acquisition |
| `chained_laps.yaml` | Several legs in a row (square, circle, zigzag, hovers, helix) |
| `helix_climb.yaml` | Helix climbing from 5 to 13 m: turning and climbing at once |
| `lissajous_fast.yaml` | Lissajous at 4 m/s, close to the 3 m/s² limit of the target |
| `takeoff_chase.yaml` | Target on the ground takes off and flees in a zigzag |
| `random_evasive.yaml` | Random route (fixed seed): abrupt direction and height changes |
| `depth_range_probe.yaml`, `roi_probe_x3.yaml` | Pursuer starts facing away; the target approaches from 34 m (ROI and depth range) |
| `tuning/s1..s4` | Controller tuning set: hover, line, figure 8, zigzag |
