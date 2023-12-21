#include "KivaGraph.h"
#include "KivaSystem.h"
#include "SIPP.h"
std::string printLocation(const BasicGraph &G, int location) {
  std::stringstream ss;
  ss << "(";
  const auto row = location / G.get_cols();
  ss << row << ",";
  const auto col = location - row * G.get_cols();
  ss << col << ")";
  return ss.str();
}

std::string printPath(const BasicGraph &G, Path const &path) {
  std::stringstream ss;
  for (auto &s : path) {
    ss << printLocation(G, s.location) << "->";
  }
  return ss.str();
}

int main(int argc, char **argv) {
  constexpr auto vehicle_cnt = 3;
  // read map
  KivaGrid G;
  auto load_map_ret = G.load_map("../maps/sipp_demo.map");
  std::cout << load_map_ret << std::endl;
  G.preprocessing(false);
  for (auto h : G.agent_home_locations) {
    std::cout << "home: " << h << ",";
    auto row = h / G.cols;
    auto col = h - G.cols * row;
    std::cout << "row: " << row << ", col: " << col << "\n";
  }
  for (auto h : G.endpoints) {
    std::cout << "endpoint: " << h << ",";
    auto row = h / G.cols;
    auto col = h - G.cols * row;
    std::cout << "row: " << row << ", col: " << col << "\n";
  }
  // reservation table
  auto rt = ReservationTable{G};
  auto paths = std::vector<Path *>{};
  paths.resize(vehicle_cnt, nullptr);
  auto sipp = SIPP{};
  rt.num_of_agents = 3;
  rt.map_size = G.size();
  rt.k_robust = 1;
  rt.window = INT_MAX;
  rt.hold_endpoints = false;

  // 从下向上，最高
  rt.clear();
  rt.build(paths, {}, {}, 2, 9 * 8 + 4);
  // State的timestep表示什么
  auto path = sipp.run(G, State{9 * 8 + 4, 0 /*timestep*/, -1},
                       std::vector<std::pair<int, int>>{{4, 0}}, rt);
  std::cout << printPath(G, path);
  paths[2] = &path;
  rt.clear();

  // 从上向下，第二
  rt.build(paths, {}, {2}, 0, 9 * 2 + 4);
  // start
  auto start0 = State{9 * 2 + 4, 0, -1};
  // goal_location <location, timestamp>
  // TODO timestamp起什么作用？
  auto goal_location0 = std::vector<std::pair<int, int>>{{7 * 9 + 4, 0}};
  auto path0 = sipp.run(G, start0, goal_location0, rt);
  std::cout << 0 << ", path: " << printPath(G, path0);
  std::cout << std::endl;
  paths[0] = &path0;

  rt.clear();
  rt.build(paths, {}, {0, 2}, 0, 9 * 4 + 7);
  auto start1 = State{4 * 9 + 7, 0, -1};
  auto goal_location1 = std::vector<std::pair<int, int>>{{4 * 9, 0}};
  auto path1 = sipp.run(G, start1, goal_location1, rt);

  std::cout << 1 << ", path: " << printPath(G, path1);
  std::cout << std::endl;

  exit(EXIT_SUCCESS);
}
