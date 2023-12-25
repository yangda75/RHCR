#include "KivaGraph.h"
#include "KivaSystem.h"
#include "PBS.h"
#include "SIPP.h"

static std::string printLocation(const BasicGraph &G, int location) {
  std::stringstream ss;
  ss << "(";
  const auto row = location / G.get_cols();
  ss << row << ",";
  const auto col = location - row * G.get_cols();
  ss << col << ")";
  return ss.str();
}

static std::string printPath(const BasicGraph &G, Path const &path) {
  std::stringstream ss;
  for (auto &s : path) {
    ss << printLocation(G, s.location) << "->";
  }
  return ss.str();
}

static int encodeLocation(BasicGraph const &g, int row, int col) {
  return row * g.cols + col;
}

namespace pbs_demo {
constexpr auto vehicle_cnt = 3;
constexpr auto simulation_time = 10;

} // namespace pbs_demo

int main(int argc, char **argv) {
  KivaGrid G;
  auto load_map_ret = G.load_map("../maps/sipp_demo.map");
  std::cout << load_map_ret << std::endl;
  auto sipp = SIPP{};
  auto pbs = PBS{G, sipp};
  KivaSystem system{G, pbs};
  G.preprocessing(false);

  // 设置参数
  // driver.cpp set_parameters
  system.outfile = "pbs_demo_output";
  system.screen = 0;
  system.log = true;
  system.num_of_drives = pbs_demo::vehicle_cnt;
  system.time_limit = 1000;
  system.simulation_window = 10;
  system.planning_window = 10;
  system.travel_time_window = 0;
  system.consider_rotation = false;
  system.k_robust = 0;
  system.hold_endpoints = false;
  system.seed = 0;
  srand(system.seed);
  // system.intialize()会随机选择起点终点
  system.initialize_solvers();
  system.paths.resize(pbs_demo::vehicle_cnt);
  // 设置起点终点
  system.starts = {
      State{encodeLocation(G, 2, 4), 0, -1},
      State{encodeLocation(G, 4, 7), 0, -1},
      State{encodeLocation(G, 8, 4), 0, -1},
  };
  system.goal_locations = std::vector<std::vector<std::pair<int, int>>>{
      {{encodeLocation(G, 7, 4), 0}},
      {{encodeLocation(G, 4, 0), 0}},
      {{encodeLocation(G, 0, 4), 0}},
  };
  system.solve();
  // system.simulate(pbs_demo::simulation_time);
  for (auto &p : system.paths) {
    std::cout << printPath(G, p) << "\n";
  }
  return 0;
}
