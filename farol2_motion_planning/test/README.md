# Planner Benchmark

A command-line benchmarking harness for `MultipleVehiclePlanner`. It runs a configured scenario sweep, collects optimization results, and writes summary and trajectory data to CSV files.

## Running

Run the compiled executable from the working directory you want to use:

```bash
./benchmarkplaner2.0
```

The default mode is `collision`, with one trial per scenario and seed `1099`. Use `--help` to list the available options:

```bash
./benchmarkplaner2.0 --help
```

| Option | Values / meaning | Default |
|---|---|---|
| `--mode` | `standard`, `compare-time`, `lower-order`, or `collision` | `collision` |
| `--trials` | Number of trials per scenario; must be greater than zero | `1` |
| `--seed` | Random-number-generator seed | `1099` |
| `--output-dir` | Base directory for generated results | `src/farol2/farol2_motion_planning/test/results` |

Examples:

```bash
./benchmarkplaner2.0 --mode standard --trials 10
./benchmarkplaner2.0 --mode lower-order --trials 5 --seed 42
./benchmarkplaner2.0 --mode collision --trials 20 --output-dir /tmp/planner-results
```

The output directory is relative to the current working directory unless an absolute path is supplied.

## Trial modes

Each mode selects a different existing trial function and its corresponding problem generator:

| Mode | Trial function | Scenario generator |
|---|---|---|
| `standard` | `runSingleTrial` | `SquareProblemGenerator` |
| `compare-time` | `runSingleTrial_comparetime` | `FormationProblemGenerator` |
| `lower-order` | `runSingleTrial_with_lower_order_guess` | `RandomProblemGenerator` |
| `collision` | `runSingleTrial_with_Collision` | `SquareProblemGenerator` |

The mode is selected at runtime, so changing modes does not require editing or commenting out trial calls in `runScenario()`.

## Configuring the sweep

The sweep is defined in `main()` in the `sweeps` vector. Edit that vector to choose which scenarios to run. Sweep helper functions include:

- `sweepVehicles(...)`
- `sweepObstacles(...)`
- `sweepDegree(...)`
- `sweepConservativeness(...)`
- `sweepDegreeSingleVehicleNoObstacles(...)`
- `sweepNsplit(...)`
- `sweepVehiclesFixedDegree(...)`

For example, the current active sweep is configured as:

```cpp
{"vehicles_deg7", sweepVehiclesFixedDegree({7}, /*fixed_degree=*/6)}
```

This runs one scenario with **7 vehicles and Bezier degree 6**. The sweep name `vehicles_deg7` is only a label; it does not set the Bezier degree.

## Output files

Results are written under:

```text
<output-dir>/<date>/<mode>/
```

For example:

```text
src/farol2/farol2_motion_planning/test/results/2026-10-13/collision/
```

For each sweep, the program writes:

- `benchmark_<date>_original_plot_<sweep>.csv` — summary statistics and box-plot values.
- `benchmark_<date>_simple_<sweep>.csv` — compact summary.
- `benchmark_<date>_controlPts_<sweep>.csv` — control points for successful main solves.
- `benchmark_<date>_trajectories_<sweep>.csv` — sampled trajectories for successful main solves.
- `benchmark_<date>_controlPts_strategy_<sweep>.csv` — strategy control points, when available.
- `benchmark_<date>_trajectories_strategy_<sweep>.csv` — strategy trajectories, when available.

Summary timing statistics include successful trials only. Failed trials are included in the failure percentage. Exceptions thrown during a trial are logged and counted as failures.

## Notes and limitations

- The trial generators and solver settings are defined in their respective trial functions. Review those functions before comparing results across modes; they do not all use the same scenario or timing method.
- `compare-time` uses a formation generator with five configured vehicles. Keep its scenario vehicle count compatible with that generator.
- `lower-order` uses randomized states and obstacles; the random scenario-generation scheme may need adjustment for the experiments you intend to run.
- In `collision` mode, the strategy tensors are only populated when the collision/failed-vehicle path performs a joint solve. Strategy CSVs may therefore have no trajectory rows for trials that need no joint solve.
- Check the CSV output after changing modes or sweep settings to confirm the expected scenarios and data were produced.