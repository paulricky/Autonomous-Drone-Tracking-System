"""Installed command-line entry points."""
from __future__ import annotations
import argparse,json
from pathlib import Path
from ece3162.mapping.world_loader import load_world
from ece3162.mapping.obstacle_inflation import inflate_obstacles
from ece3162.simulation.runner import run_navigation,run_dynamic_replanning

ROOT=Path(__file__).resolve().parents[2]
def _run()->int:
    parser=argparse.ArgumentParser();parser.add_argument("--scenario",choices=("basic","static","dynamic"),default="static");parser.add_argument("--output",default="results/advanced_navigation");args=parser.parse_args()
    world_name={"basic":"duckiedrone_baseline","static":"advanced_static","dynamic":"advanced_dynamic"}[args.scenario]
    world=load_world(ROOT/"simulation/worlds"/world_name/"world.yaml");inflated=inflate_obstacles(world.voxel_map,.2,.3)
    dynamic=None
    if args.scenario=="dynamic":
        import numpy as np
        dynamic=run_dynamic_replanning(inflated,world.start,world.goal,(np.array([2.,-.5,0.]),np.array([3.,.5,2.5])));run=dynamic.navigation
    else:run=run_navigation(inflated,world.start,world.goal,world.obstacle_boxes)
    metrics=run.metrics.to_dict()
    if dynamic:metrics.update(replanning_latency_s=dynamic.replanning_latency_s,replan_count=dynamic.replan_count,obstacle_appeared_at_s=dynamic.obstacle_appeared_at_s)
    out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True);(out/f"{args.scenario}_metrics.json").write_text(json.dumps(metrics,indent=2)+"\n")
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(15,6));ax=fig.add_subplot(121,projection="3d");err=fig.add_subplot(122)
    ax.plot(*run.global_positions.T,"o--",label="kinodynamic A*");ax.plot(*run.reference_positions.T,label="optimized B-spline");ax.plot(*run.executed_positions.T,label="executed");ax.scatter(*world.start,s=80,label="start");ax.scatter(*world.goal,s=120,marker="*",label="goal")
    ax.set(xlabel="x (m)",ylabel="y (m)",zlabel="z (m)",title=f"{args.scenario} 3-D navigation");ax.legend()
    error_times=run.times[:len(run.position_errors)];err.plot(error_times,run.position_errors,label="position");err.plot(error_times,run.velocity_errors,label="velocity");err.set(xlabel="time (s)",ylabel="error",title="Feedback tracking error");err.grid();err.legend();fig.tight_layout();fig.savefig(out/f"{args.scenario}_navigation.png",dpi=180);plt.close(fig)
    print(json.dumps(metrics,indent=2));return 0 if run.metrics.success and run.metrics.collision_count==0 else 1
def simulation_main()->int:return _run()
def benchmark_main()->int:return _run()

if __name__=="__main__":
    raise SystemExit(_run())
