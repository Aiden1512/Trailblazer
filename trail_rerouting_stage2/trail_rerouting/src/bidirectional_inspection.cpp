#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct Point { double x{}, y{}; };
struct Trail { int id{}, a{}, b{}; std::vector<Point> polyline; double length{}; bool inspected{false}; bool blocked{false}; double fraction{0.5}; bool inspected_a{false}, inspected_b{false}; };
struct Hop { int trail{}, from{}, to{}; };
static double distance(const Point& a, const Point& b) { return std::hypot(a.x-b.x, a.y-b.y); }

class Coverage {
 public:
  void load(const std::string& file) {
    const auto root = YAML::LoadFile(file);
    for (const auto& j : root["junctions"]) {
      const int id=j["id"].as<int>();
      if (!nodes.emplace(id, Point{j["x"].as<double>(), j["y"].as<double>()}).second)
        throw std::runtime_error("Duplicate junction ID");
    }
    for (const auto& t : root["trails"]) {
      const int id=t["id"].as<int>();
      if (!t["end_junction"] || t["end_junction"].IsNull() || !t["waypoints"] || t["waypoints"].size()==0) {
        std::cout << "Skipping endpoint/incomplete trail " << id << "\n";
        continue;
      }
      Trail e; e.id=id; e.a=t["start_junction"].as<int>(); e.b=t["end_junction"].as<int>();
      if (!nodes.count(e.a) || !nodes.count(e.b)) throw std::runtime_error("Unknown junction for trail " + std::to_string(id));
      e.polyline.push_back(nodes.at(e.a));
      for (const auto& p : t["waypoints"]) e.polyline.push_back({p["x"].as<double>(), p["y"].as<double>()});
      e.polyline.push_back(nodes.at(e.b));
      for (size_t i=1;i<e.polyline.size();++i) e.length+=distance(e.polyline[i-1],e.polyline[i]);
      if (e.length<=0 || !trails.emplace(id,e).second) throw std::runtime_error("Invalid or duplicate trail " + std::to_string(id));
    }
  }
  void block(int id,double fraction) {
    auto it=trails.find(id);
    if(it==trails.end()) throw std::runtime_error("Unknown trail " + std::to_string(id));
    if (!(fraction>0.0 && fraction<1.0)) throw std::runtime_error("Obstacle fraction must be between 0 and 1 (exclusive)");
    it->second.blocked=true; it->second.fraction=fraction;
  }
  std::vector<Hop> shortest(int from,int to) const {
    if (!nodes.count(from)||!nodes.count(to)) throw std::runtime_error("Unknown junction");
    std::map<int,double> dist;
    std::map<int,Hop> prev;
    for (const auto& [id,p]:nodes) dist[id]=std::numeric_limits<double>::infinity();
    using Q=std::pair<double,int>;
    std::priority_queue<Q,std::vector<Q>,std::greater<Q>> pq;
    dist[from]=0; pq.push({0,from});
    while(!pq.empty()) {
      const auto [d,u]=pq.top(); pq.pop();
      if(d>dist[u]) continue;
      if(u==to) break;
      for(const auto& [id,e]:trails) {
        if(e.blocked || (e.a!=u && e.b!=u)) continue;
        const int v=e.a==u?e.b:e.a;
        if(d+e.length<dist[v]) { dist[v]=d+e.length; prev[v]={id,u,v}; pq.push({dist[v],v}); }
      }
    }
    if(!std::isfinite(dist[to])) return {};
    std::vector<Hop> path;
    for(int v=to;v!=from;) { const Hop h=prev.at(v); path.push_back(h); v=h.from; }
    std::reverse(path.begin(),path.end()); return path;
  }
  double length(const std::vector<Hop>& path) const {
    double total=0; for(const auto& h:path) total+=trails.at(h.trail).length; return total;
  }
  Point atFraction(const Trail& t,double f) const {
    double target=t.length*f;
    for(size_t i=1;i<t.polyline.size();++i) {
      const double segment=distance(t.polyline[i-1],t.polyline[i]);
      if(target<=segment || i==t.polyline.size()-1) {
        const double alpha=segment>0?std::clamp(target/segment,0.0,1.0):0.0;
        return {t.polyline[i-1].x+alpha*(t.polyline[i].x-t.polyline[i-1].x),
                t.polyline[i-1].y+alpha*(t.polyline[i].y-t.polyline[i-1].y)};
      }
      target-=segment;
    }
    return t.polyline.back();
  }
  void run(int start) {
    if(!nodes.count(start)) throw std::runtime_error("Unknown start junction");
    int current=start;
    std::cout<<std::fixed<<std::setprecision(2);
    std::cout<<"Map: "<<nodes.size()<<" junctions, "<<trails.size()<<" navigable trails\n";
    for(const auto& [id,t]:trails) if(t.blocked) {
      const Point p=atFraction(t,t.fraction);
      std::cout<<"SIMULATED OBSTACLE trail "<<id<<" at "<<t.fraction*100<<"% from J"<<t.a
               <<" near ("<<p.x<<", "<<p.y<<")\n";
    }
    // Greedily visit each reachable uninspected trail or blocked-side inspection task.
    // No blocked trail is ever used as a through connection.
    while(true) {
      double best=std::numeric_limits<double>::infinity();
      int chosen=-1,entry=-1,exit=-1; bool side=false;
      std::vector<Hop> approach;
      for(const auto& [id,t]:trails) {
        if(!t.blocked && t.inspected) continue;
        for(const auto [u,v]: {std::pair<int,int>{t.a,t.b}, {t.b,t.a}}) {
          if(t.blocked && ((u==t.a && t.inspected_a)||(u==t.b && t.inspected_b))) continue;
          auto path=shortest(current,u);
          if(current!=u && path.empty()) continue;
          const double cost=length(path)+(t.blocked ? 2*t.length*(u==t.a?t.fraction:1-t.fraction) : t.length);
          if(cost<best) { best=cost; chosen=id; entry=u; exit=v; side=t.blocked; approach=std::move(path); }
        }
      }
      if(chosen<0) break;
      for(const auto& h:approach) std::cout<<"TRANSIT trail "<<h.trail<<" J"<<h.from<<" -> J"<<h.to<<"\n";
      current=entry;
      auto& t=trails.at(chosen);
      if(side) {
        const double fraction_from_entry=entry==t.a?t.fraction:1-t.fraction;
        std::cout<<"INSPECT BLOCKED SIDE trail "<<chosen<<" from J"<<entry
                 <<" toward obstacle ("<<fraction_from_entry*100<<"% of trail); STOP SHORT and RETURN to J"<<entry<<"\n";
        if(entry==t.a) t.inspected_a=true; else t.inspected_b=true;
      } else {
        std::cout<<"INSPECT trail "<<chosen<<" J"<<entry<<" -> J"<<exit<<"\n";
        t.inspected=true; current=exit;
      }
    }
    const auto home=shortest(current,start);
    if(current!=start && home.empty()) std::cout<<"WARNING: no safe return route to J"<<start<<"\n";
    else {
      for(const auto& h:home) std::cout<<"RETURN trail "<<h.trail<<" J"<<h.from<<" -> J"<<h.to<<"\n";
      std::cout<<"MISSION END at J"<<start<<"\n";
    }
    std::cout<<"\nINSPECTION SUMMARY\n";
    for(const auto& [id,t]:trails) {
      std::cout<<"Trail "<<id<<": ";
      if(t.inspected) std::cout<<"FULLY INSPECTED";
      else if(t.blocked) std::cout<<"BLOCKED; J"<<t.a<<" side "<<(t.inspected_a?"attempted":"unreachable")
                                  <<", J"<<t.b<<" side "<<(t.inspected_b?"attempted":"unreachable")
                                  <<"; obstacle not crossed";
      else std::cout<<"UNINSPECTED / UNREACHABLE";
      std::cout<<"\n";
    }
    std::cout<<"NOTE: This is a PLAN ONLY. Simulated side inspection does not prove obstacle clearance or actual robot coverage.\n";
  }
 private:
  std::map<int,Point> nodes;
  std::map<int,Trail> trails;
};
int main(int argc,char** argv) {
  try {
    if(argc!=3 && argc!=5) {
      std::cerr<<"Usage: bidirectional_inspection MAP.yaml START_JUNCTION [BLOCKED_TRAIL_ID OBSTACLE_FRACTION]\n"
               <<"Example: bidirectional_inspection junction_nodes.yaml 1 2 0.5\n";
      return 1;
    }
    Coverage c; c.load(argv[1]);
    if(argc==5) c.block(std::stoi(argv[3]),std::stod(argv[4]));
    c.run(std::stoi(argv[2]));
  } catch(const std::exception& e) { std::cerr<<"Planner error: "<<e.what()<<"\n"; return 1; }
}
