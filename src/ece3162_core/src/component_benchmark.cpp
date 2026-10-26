#include "ece3162_core/ego_optimizer.hpp"
#include "ece3162_core/kinodynamic_astar.hpp"
#include <chrono>
#include <iostream>
#include <string>
using namespace ece3162::navigation;
template<class F>double timed(F&&f){auto a=std::chrono::steady_clock::now();f();return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-a).count();}
int main(int argc,char**argv){int trials=argc>1?std::stoi(argv[1]):30;VoxelMap map({-1,-2,0},{6,2,3},.25,Occupancy::kFree);std::vector<Vec3>guide{{0,0,1},{.7,.2,1},{1.4,.4,1},{2.1,.2,1},{3,0,1}};BSpline spline(guide,5);volatile double sink=0;std::cout<<"component,trial,time_ms\n";for(int trial=-2;trial<trials;++trial){double t=timed([&]{auto r=KinodynamicAStar({.5,1,1,.5,0,.35,.55,.05,1,3,20000,false}).plan({{0,0,1},{0,0,0},0},{3,0,1},map);sink+=r.cost;});if(trial>=0)std::cout<<"kinodynamic_astar,"<<trial<<","<<t<<"\n";t=timed([&]{for(int i=0;i<10000;++i)sink+=spline.position(5.0*i/9999)[0];});if(trial>=0)std::cout<<"bspline_10000,"<<trial<<","<<t<<"\n";t=timed([&]{for(int i=0;i<10000;++i)sink+=map.occupied({3.0*i/9999,0,1});});if(trial>=0)std::cout<<"collision_10000,"<<trial<<","<<t<<"\n";t=timed([&]{auto r=EgoOptimizer().optimize(guide,map,5,{2,2});sink+=r.final.total();});if(trial>=0)std::cout<<"ego_optimizer,"<<trial<<","<<t<<"\n";}return sink<0;}
