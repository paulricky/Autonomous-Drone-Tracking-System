# Duckiedrone software-only simulation

Install the existing project environment, then run:

```bash
python -m pytest -q
python -m ece3162.cli --scenario basic
python -m ece3162.cli --scenario static
python -m ece3162.cli --scenario dynamic
MPLBACKEND=Agg python scripts/run_duckiedrone_sim.py --scenario baseline
MPLBACKEND=Agg python scripts/run_duckiedrone_sim.py --scenario obstacle
```

Use `--output-dir PATH` to change results and `--show` for interactive plots.
Defaults are in `config/duckiedrone_simulation.yaml`; hardware publication is
false and the runner rejects a configuration that enables it.

The loop is A* → collision-checked shortened path → waypoint state machine →
proportional velocity demand → speed and acceleration limiting → integrated
position → fresh feedback. It has nonzero transient error and never teleports
the state to a waypoint. It is a simplified point-mass integration test, not a
quadcopter attitude, rotor, aerodynamics, or flight-safety validation.

The baseline world is obstacle-free at constant 1 m altitude. The obstacle
world reuses the repository's 3-D multi-obstacle world. Reported clearance is
the distance from the simulated vehicle center to obstacle boxes minus the
configured spherical drone radius; collision count is the number of sampled
states intersecting that physical envelope. Planning uses the additionally
inflated map (drone radius plus safety margin).

Tracking error is the instantaneous distance to the currently active waypoint,
so its maximum includes the initial distance to a newly issued waypoint. A
world with no obstacles reports minimum clearance as `null` (not applicable).

IAPF is unavailable because no IAPF/MATLAB implementation is present. The
runner intentionally offers only the verified A* option.
