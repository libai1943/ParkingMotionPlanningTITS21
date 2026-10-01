#pragma once
// LIOM: scene input, hybrid A* initialization and safe travel corridors.
#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
extern "C" int plan_rigid_path(const double *, const double *, const double *,
                               const double *, int, int, double *, int, char *,
                               int);
using casadi::DM;
using casadi::SX;
namespace fs = std::filesystem;
constexpr int nodes = 200, fields = 11;
constexpr double wheelbase = 2.8, rear_center = .24325, front_center = 2.58775;
using Pose = std::array<double, 3>;
using Box = std::array<double, 4>;
struct Scene {
  std::array<double, 6> boundary;
  std::vector<double> obstacles;
  std::vector<std::vector<int>> map;
};
Scene load_scene(const fs::path &path) {
  Scene s;
  std::ifstream input(path / "case_1.txt");
  int count = 0;
  for (auto &q : s.boundary)
    input >> q;
  input >> count;
  s.obstacles.resize(8 * count);
  for (auto &q : s.obstacles)
    input >> q;
  if (!input || count < 1)
    throw std::runtime_error("Cannot read case_1.txt");
  std::ifstream grid(path / "dilated_map.csv");
  std::string line;
  while (std::getline(grid, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    std::vector<int> values;
    int v;
    while (row >> v)
      values.push_back(v);
    s.map.push_back(values);
  }
  if (s.map.size() != 125)
    throw std::runtime_error("Expected the original 125 x 125 dilated map");
  for (auto &row : s.map)
    if (row.size() != 125)
      throw std::runtime_error("Invalid map width");
  return s;
}
bool free_box(const Scene &s, const Box &b) {
  if (b[0] < -20 + std::hypot(4.689 / 4, 1.942 / 2) ||
      b[1] > 20 - std::hypot(4.689 / 4, 1.942 / 2) ||
      b[2] < -20 + std::hypot(4.689 / 4, 1.942 / 2) ||
      b[3] > 20 - std::hypot(4.689 / 4, 1.942 / 2))
    return false;
  int xl = int(std::floor((b[0] + 20) / .32)),
      xu = int(std::floor((b[1] + 20) / .32));
  int yl = int(std::floor((b[2] + 20) / .32)),
      yu = int(std::floor((b[3] + 20) / .32));
  for (int x = xl; x <= xu; ++x)
    for (int y = yl; y <= yu; ++y)
      if (s.map[x][y])
        return false;
  return true;
}
Box grow_box(const Scene &s, double x, double y) {
  if (!free_box(s, {x, x, y, y})) {
    bool found = false;
    for (int step = 1; step <= 4000 && !found; ++step) {
      int angle_id = step % 8;
      double radius = (1 + (step - angle_id) / 8) * .16;
      double angle = angle_id * std::acos(-1.) / 4;
      double xx = x + radius * std::cos(angle),
             yy = y + radius * std::sin(angle);
      if (free_box(s, {xx, xx, yy, yy})) {
        x = xx;
        y = yy;
        found = true;
        break;
      }
    }
    if (!found)
      throw std::runtime_error("No free corridor seed");
  }
  Box b{x, x, y, y};
  std::array<double, 4> length{};
  std::array<bool, 4> done{};
  // The original order is up, left, down, right, with 0.1 m increments.
  std::array<int, 4> side{3, 0, 2, 1};
  int remaining = 4;
  while (remaining) {
    for (int d = 0; d < 4; ++d)
      if (!done[d]) {
        int k = side[d];
        Box proposed = b;
        proposed[k] += (k == 0 || k == 2 ? -.1 : .1);
        if (length[d] + .1 > 2.0 || !free_box(s, proposed)) {
          done[d] = true;
          --remaining;
        } else {
          b = proposed;
          length[d] += .1;
        }
      }
  }
  return b;
}
std::vector<Pose> search(const Scene &s) {
  double bounds[] = {-20, 20, -20, 20},
         vehicle[] = {2.8, 3.76, .929, 1.942, .75};
  std::vector<double> buffer(300000);
  char message[512]{};
  int n = plan_rigid_path(bounds, vehicle, s.boundary.data(),
                          s.obstacles.data(), int(s.obstacles.size() / 8), 1,
                          buffer.data(), 100000, message, 512);
  if (n < 2)
    throw std::runtime_error(message);
  std::vector<Pose> path(n);
  for (int i = 0; i < n; ++i)
    for (int k = 0; k < 3; ++k)
      path[i][k] = buffer[i * 3 + k];
  return path;
}
std::vector<Pose> resample(const std::vector<Pose> &path, double &length) {
  std::vector<double> distance(path.size(), 0);
  for (size_t i = 1; i < path.size(); ++i)
    distance[i] = distance[i - 1] + std::hypot(path[i][0] - path[i - 1][0],
                                               path[i][1] - path[i - 1][1]);
  length = distance.back();
  std::vector<Pose> sampled(nodes);
  size_t j = 1;
  for (int i = 0; i < nodes; ++i) {
    double s = length * i / (nodes - 1);
    while (j + 1 < path.size() && distance[j] < s)
      ++j;
    double a =
        (s - distance[j - 1]) / std::max(1e-12, distance[j] - distance[j - 1]);
    for (int k = 0; k < 3; ++k)
      sampled[i][k] = path[j - 1][k] + a * (path[j][k] - path[j - 1][k]);
  }
  return sampled;
}
