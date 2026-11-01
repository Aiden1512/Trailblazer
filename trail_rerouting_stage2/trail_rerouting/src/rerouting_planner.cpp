#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

struct Point { double x{}, y{}, yaw{}; };
struct Trail {
  int id{}, from{}, to{};
  std::vector<Point> waypoints;
  double length{};
  bool blocked{false}, inspected{false};
};
struct Step { int trail_id{}, from{}, to{}; bool inspection{}; };

class Planner {
 public:
  void load(const std::string& filename) {
    auto root = YAML::LoadFile(filename);
    frame_ = root["frame_id"].as<std::string>();
    for (const auto& j : root["junctions"]) {
      const int id = j["id"].as<int>();
      if (!junctions_.emplace(id, Point{j["x"].as<double>(), j["y"].as<double>(), j["yaw"].as<double>()}).second)
        throw std::runtime_error("Duplicate junction ID " + std::to_string(id));
    }
    for (const auto& t : root["trails"]) {
      Trail e;
      e.id = t["id"].as<int>();
      e.from = t["start_junction"].as<int>();
      if (!junctions_.count(e.from)) throw std::runtime_error("Unknown start junction");
      if (!t["end_junction"] || t["end_junction"].IsNull() || !t["waypoints"] || t["waypoints"].size() == 0) {
        std::cout << "Skipping incomplete trail " << e.id << "\n";
        continue;
      }
      e.to = t["end_junction"].as<int>();
      if (!junctions_.count(e.to)) throw std::runtime_error("Unknown end junction");
      for (const auto& p : t["waypoints"])
        e.waypoints.push_back({p["x"].as<double>(), p["y"].as<double>(), p["yaw"].as<double>()});
      Point prev = junctions_.at(e.from);
      for (const auto& p : e.waypoints) { e.length += distance(prev, p); prev = p; }
      e.length += distance(prev, junctions_.at(e.to));
      if (!trails_.emplace(e.id, std::move(e)).second)
        throw std::runtime_error("Duplicate trail ID");
    }
  }

  void block(int id) {
    auto it = trails_.find(id);
    if (it == trails_.end()) throw std::runtime_error("Unknown/incomplete trail " + std::to_string(id));
    it->second.blocked = true;
  }

  // Dijkstra over an undirected MULTIGRAPH; each trail ID is an independent edge.
  std::vector<Step> shortest(int start, int goal) const {
    if (!junctions_.count(start) || !junctions_.count(goal)) throw std::runtime_error("Unknown junction");
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    std::map<int, double> dist;
    std::map<int, std::pair<int,int>> parent; // node -> {previous node, trail ID}
    for (const auto& j : junctions_) dist[j.first] = std::numeric_limits<double>::infinity();
    dist[start] = 0; pq.push({0, start});
    while (!pq.empty()) {
      auto [d,u] = pq.top(); pq.pop();
      if (d > dist[u]) continue;
      if (u == goal) break;
      for (const auto& [id,e] : trails_) {
        if (e.blocked || (e.from != u && e.to != u)) continue;
        int v = e.from == u ? e.to : e.from;
        double candidate = d + e.length;
        if (candidate < dist[v]) {
          dist[v] = candidate; parent[v] = {u,id}; pq.push({candidate,v});
        }
      }
    }
    if (!std::isfinite(dist[goal])) return {};
    std::vector<Step> result;
    for (int v = goal; v != start;) {
      auto [u,id] = parent.at(v);
      result.push_back({id,u,v,false}); v = u;
    }
    std::reverse(result.begin(), result.end());
    return result;
  }

  // Greedy coverage: shortest repositioning path to an endpoint of an uninspected trail.
  // This is a baseline, not a globally optimal route-inspection solver.
  std::vector<Step> coverage(int start) {
    if (!junctions_.count(start)) throw std::runtime_error("Unknown starting junction");
    std::vector<Step> route;
    int current = start;
    while (true) {
      double best_cost = std::numeric_limits<double>::infinity();
      int chosen = -1, entry = -1, exit = -1;
      std::vector<Step> approach;
      for (const auto& [id,e] : trails_) {
        if (e.blocked || e.inspected) continue;
        for (auto [a,b] : {std::pair<int,int>{e.from,e.to}, {e.to,e.from}}) {
          auto path = shortest(current,a);
          if (current != a && path.empty()) continue;
          double cost = e.length;
          for (const auto& s : path) cost += trails_.at(s.trail_id).length;
          if (cost < best_cost) {
            best_cost = cost; chosen = id; entry = a; exit = b; approach = path;
          }
        }
      }
      if (chosen < 0) break;
      route.insert(route.end(), approach.begin(), approach.end());
      route.push_back({chosen,entry,exit,true});
      trails_.at(chosen).inspected = true;
      current = exit;
    }
    return route;
  }

  void report() const {
    std::cout << "Frame: " << frame_ << ", junctions: " << junctions_.size()
              << ", usable trails: " << trails_.size() << "\n";
    for (const auto& [id,t] : trails_)
      std::cout << "Trail " << id << ": " << t.from << " <-> " << t.to
                << ", length " << t.length << "m" << (t.blocked ? " [BLOCKED]" : "") << "\n";
  }
  void uninspectedReport() const {
    for (const auto& [id,t] : trails_)
      if (!t.inspected) std::cout << "Uninspected trail " << id
                                  << (t.blocked ? " (blocked)" : " (unreachable)") << "\n";
  }
 private:
  static double distance(const Point& a, const Point& b) { return std::hypot(a.x-b.x,a.y-b.y); }
  std::string frame_;
  std::map<int,Point> junctions_;
  std::map<int,Trail> trails_;
};

int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      std::cerr << "Usage: rerouting_planner MAP.yaml START_JUNCTION [BLOCKED_TRAIL_ID ...]\n";
      return 1;
    }
    Planner planner;
    planner.load(argv[1]);
    const int start = std::stoi(argv[2]);
    for (int i=3;i<argc;++i) planner.block(std::stoi(argv[i]));
    planner.report();
    const auto route = planner.coverage(start);
    std::cout << "\nPlanned traversal:\n";
    for (const auto& step : route)
      std::cout << (step.inspection ? "INSPECT " : "TRANSIT ") << "trail " << step.trail_id
                << " from J" << step.from << " to J" << step.to << "\n";
    std::cout << "\nRemaining trails:\n";
    planner.uninspectedReport();
  } catch (const std::exception& e) {
    std::cerr << "Planner error: " << e.what() << "\n";
    return 1;
  }
}
