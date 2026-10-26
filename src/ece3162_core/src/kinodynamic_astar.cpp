#include "ece3162_core/kinodynamic_astar.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace ece3162::navigation {
namespace {
struct Key { int p[3],v[3]; bool operator==(const Key&o)const noexcept{for(int i=0;i<3;++i)if(p[i]!=o.p[i]||v[i]!=o.v[i])return false;return true;} };
struct Hash {std::size_t operator()(const Key&k)const noexcept{std::size_t h=1469598103934665603ULL;for(int x:k.p){h^=static_cast<std::size_t>(x);h*=1099511628211ULL;}for(int x:k.v){h^=static_cast<std::size_t>(x);h*=1099511628211ULL;}return h;}};
struct Node {PlanningState state;double g{};Key parent{};Vec3 control{};bool has_parent{false};};
struct Queue {double f;std::size_t sequence;Key key;bool operator<(const Queue&o)const noexcept{return f>o.f||(f==o.f&&sequence>o.sequence);}};
}
KinodynamicAStar::KinodynamicAStar(KinodynamicConfig c):config_(c){if(c.primitive_duration<=0||c.max_velocity<=0||c.max_acceleration<=0||c.velocity_resolution<=0||c.collision_sample_dt<=0||c.timeout_s<=0||c.goal_position_tolerance<0||c.goal_velocity_tolerance<0)throw std::invalid_argument("invalid kinodynamic configuration");}
PlanningState KinodynamicAStar::propagate(const PlanningState&s,const MotionPrimitive&u){if(!finite(s.position)||!finite(s.velocity)||!finite(u.acceleration)||u.duration<=0)throw std::invalid_argument("invalid propagation input");return {add(add(s.position,mul(s.velocity,u.duration)),mul(u.acceleration,.5*u.duration*u.duration)),add(s.velocity,mul(u.acceleration,u.duration)),s.stamp+u.duration};}
PlannerResult KinodynamicAStar::plan(const PlanningState&start,const Vec3&goal,const VoxelMap&map,Vec3 terminal)const{
  const auto begun=std::chrono::steady_clock::now();PlannerResult out;auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-begun).count();};
  if(!finite(start.position)||!finite(start.velocity)||!finite(goal)||!finite(terminal)){out.message="non-finite planning input";return out;}
  if(map.occupied(start.position)){out.code=FailureCode::kStartCollision;out.message="start in collision";return out;}if(map.occupied(goal)){out.code=FailureCode::kGoalCollision;out.message="goal in collision";return out;}if(norm(start.velocity)>config_.max_velocity+1e-9){out.code=FailureCode::kInitialVelocity;out.message="initial velocity exceeds limit";return out;}
  const double pr=config_.position_resolution>0?config_.position_resolution:map.resolution();auto key=[&](const PlanningState&s){Key k{};for(int i=0;i<3;++i){k.p[i]=static_cast<int>(std::floor((s.position[i]-map.minimum()[i])/pr));k.v[i]=static_cast<int>(std::llround(s.velocity[i]/config_.velocity_resolution));}return k;};
  auto h=[&](const Vec3&p){return std::max(0.0,norm(sub(goal,p))-config_.goal_position_tolerance)/config_.max_velocity;};
  std::vector<Vec3> controls{{0,0,0}};for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){int nonzero=(x!=0)+(y!=0)+(z!=0);if(nonzero==0||(!config_.diagonal_accelerations&&nonzero>1))continue;Vec3 a{double(x),double(y),double(z)};a=mul(a,config_.max_acceleration/norm(a));controls.push_back(a);}
  std::priority_queue<Queue> open;std::unordered_map<Key,Node,Hash> nodes;std::unordered_set<Key,Hash> closed;std::size_t seq=0;const Key sk=key(start);nodes.emplace(sk,Node{start,0});open.push({h(start.position),seq++,sk});Key found{};bool success=false;
  while(!open.empty()&&closed.size()<config_.max_expansions){if(elapsed()>config_.timeout_s){out.code=FailureCode::kTimeout;out.message="planning timeout";break;}auto q=open.top();open.pop();if(closed.count(q.key))continue;closed.insert(q.key);const Node current=nodes.at(q.key);if(norm(sub(current.state.position,goal))<=config_.goal_position_tolerance&&norm(sub(current.state.velocity,terminal))<=config_.goal_velocity_tolerance){found=q.key;success=true;break;}
    for(const auto&a:controls){const auto next=propagate(current.state,{a,config_.primitive_duration});if(norm(next.velocity)>config_.max_velocity+1e-9||!map.contains(next.position))continue;bool collision=false;const auto samples=std::max(2,static_cast<int>(std::ceil(config_.primitive_duration/config_.collision_sample_dt)));for(int i=1;i<=samples;++i){double t=config_.primitive_duration*i/samples;Vec3 p=add(add(current.state.position,mul(current.state.velocity,t)),mul(a,.5*t*t));++out.collision_checks;if(map.occupied(p)){collision=true;break;}}if(collision)continue;const Key nk=key(next);if(closed.count(nk))continue;const double ng=current.g+config_.primitive_duration;auto it=nodes.find(nk);if(it!=nodes.end()&&ng+1e-12>=it->second.g)continue;Node n{next,ng,q.key,a,true};nodes[nk]=n;open.push({ng+config_.heuristic_weight*h(next.position),seq++,nk});++out.generated;}
  }
  out.expanded=closed.size();out.planning_time_s=elapsed();if(!success){if(out.message.empty()){out.code=closed.size()>=config_.max_expansions?FailureCode::kExpansionLimit:FailureCode::kUnreachable;out.message=out.code==FailureCode::kExpansionLimit?"expansion limit":"unreachable";}return out;}
  std::vector<PlanningState> states;std::vector<Vec3> actions;for(Key k=found;;){const auto&n=nodes.at(k);states.push_back(n.state);if(!n.has_parent)break;actions.push_back(n.control);k=n.parent;}out.states.assign(states.rbegin(),states.rend());out.accelerations.assign(actions.rbegin(),actions.rend());out.cost=nodes.at(found).g;out.success=true;out.code=FailureCode::kNone;out.message="trajectory found";return out;
}
}  // namespace ece3162::navigation
