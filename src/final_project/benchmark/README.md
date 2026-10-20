# Benchmark (camera without gimbal)

The follower is the system under test: its limits, gains, follow distance, timeouts and
camera stay as in `config.py` for every run. Only the leader changes, through the
overrides listed in `scenarios.py`.

| Phase | What it tests | Runs |
|---|---|---|
| A | hover, straight line at 3 / 6 / 10 m/s | 4 |
| B | circle R = 10 m at 3 / 5 / 7 m/s (limit ≈ 5.1 m/s), flat figure-8 at 2 / 3 m/s | 5 |
| C | altitude step of 2 / 5 m, figure-8 with H = 5 m | 3 |
| D | random waypoints with stops, 4 leader levels x 3 seeds | 12 |
| E | far spawn: exploration and search, levels 1-2 x 2 seeds | 4 |

Leader levels (maneuver limits): 1 = 5 m/s, 1.5 m/s² · 2 = 8 m/s, 3 m/s² ·
3 = 12 m/s, 5 m/s² · 4 = 15 m/s, 8 m/s². Follower tracking acceleration: 2.6 m/s² (tilt 15°).

## Running

Inside the tk3lab container, with no simulation running (the runner starts and stops
`simulation/quad_simulation.sh` for every scenario):

```sh
cd ~/tk3lab-ws/src/final_project/benchmark
python3 run_benchmark.py --campaign c1 --only A01          # first a single short run
python3 run_benchmark.py --campaign c1 --resume            # then the whole campaign (about 1 hour)
python3 analyze_benchmark.py results/c1
```

- `--phase A B` or `--only D21 D22` run a subset; `--resume` skips the runs already completed.
- Before each run the runner cleans up what a previous simulation may have left (`gz sim`, pocolibs
  devices), then starts the main as soon as `sim.log` shows the 9 components running and both drones
  created in Gazebo, plus `--settle 2` seconds. If the main fails within 30 s it retries once with a
  longer settle time; the logs of the failed attempt are kept as `failed_main.log` / `failed_sim.log`.
- Each run takes about 20 s more than its trajectory: about 3 s of motor start and 6 s of climb to 5 m,
  1 s of hover, 5 s of final tail, plus the simulation start and stop.
- Each run is in `results/<campaign>/<id>/`: `main.log`, `sim.log`, `log/pom*.log`, `overrides.json`.
- The report is in `results/<campaign>/report/`: `summary.csv`, `summary.md`, `phase_*.png`,
  `runs/<id>.png` (the `tracking_plotter.py` figure of each run). `results/` is not committed.
