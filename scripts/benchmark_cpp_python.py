#!/usr/bin/env python3
"""Equivalent-workload Python/C++ component benchmark; writes raw and summary JSON."""
import argparse,csv,json,platform,subprocess,time
from pathlib import Path
import numpy as np
from ece3162.mapping.voxel_map import VoxelMap
from ece3162.planning.collision_checker import CollisionChecker
from ece3162.planning.kinodynamic_astar import KinodynamicAStar,KinodynamicConfig
from ece3162.planning.ego_planner import EgoInspiredOptimizer,EgoConfig
from ece3162.trajectory.bspline import BSplineTrajectory

def stats(v):
    a=np.asarray(v);return {"median_ms":float(np.median(a)),"mean_ms":float(a.mean()),"stddev_ms":float(a.std()),"p95_ms":float(np.percentile(a,95)),"p99_ms":float(np.percentile(a,99)),"maximum_ms":float(a.max())}
def main():
    p=argparse.ArgumentParser();p.add_argument("--cpp",required=True);p.add_argument("--trials",type=int,default=30);p.add_argument("--output",default="results/advanced_navigation/native_benchmark.json");a=p.parse_args()
    raw={"python":{},"cpp":{}};m=VoxelMap((-1,-2,0),(6,2,3),.25);m.mark_all_free();guide=np.array([[0,0,1],[.7,.2,1],[1.4,.4,1],[2.1,.2,1],[3,0,1.]])
    spline=BSplineTrajectory(guide,5);checker=CollisionChecker(m);cfg=KinodynamicConfig(primitive_duration=.5,max_velocity=1,max_acceleration=1,timeout=3,max_expansions=20000);opt=EgoInspiredOptimizer(EgoConfig())
    jobs={"kinodynamic_astar":lambda:KinodynamicAStar(cfg).plan([0,0,1],[3,0,1],m),"bspline_10000":lambda:[spline.position(5*i/9999) for i in range(10000)],"collision_10000":lambda:[checker.point_in_collision((3*i/9999,0,1)) for i in range(10000)],"ego_optimizer":lambda:opt.optimize(guide,m,5,2,2)}
    for name,fn in jobs.items():
        for _ in range(2):fn()
        vals=[]
        for _ in range(a.trials):t=time.perf_counter();fn();vals.append((time.perf_counter()-t)*1000)
        raw["python"][name]=vals
    output=subprocess.check_output([a.cpp,str(a.trials)],text=True);rows=csv.DictReader(output.splitlines())
    for row in rows:raw["cpp"].setdefault(row["component"],[]).append(float(row["time_ms"]))
    summary={};
    for name in jobs:
        summary[name]={"python":stats(raw["python"][name]),"cpp":stats(raw["cpp"][name])};summary[name]["speedup_median"]=summary[name]["python"]["median_ms"]/summary[name]["cpp"]["median_ms"]
    result={"method":{"trials":a.trials,"warmups":2,"host":platform.platform(),"python":platform.python_version(),"same_host":True,"cpp_flags":"-O3 -DNDEBUG"},"summary":summary,"raw_ms":raw}
    path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2)+"\n");print(json.dumps(result["summary"],indent=2))
if __name__=="__main__":main()
