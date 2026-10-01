// LIOM, Algorithm 3 and equations (17)--(22), with the repository's NLP.mod
// transcription.
#include "planning.hpp"

std::vector<double> initial_guess(const std::vector<Pose> &ref, double length) {
  std::vector<double> z(fields * nodes + 1, 0);
  z.back() = std::clamp(length / 1.5 + 5., 10., 90.);
  double h = z.back() / (nodes - 1);
  for (int i = 0; i < nodes; ++i) {
    for (int j = 0; j < 3; ++j)
      z[i * fields + j] = ref[i][j];
    if (i > 0 && i < nodes - 1) {
      double ds = (ref[i][0] - ref[i - 1][0]) * cos(ref[i][2]) +
                  (ref[i][1] - ref[i - 1][1]) * sin(ref[i][2]);
      z[i * fields + 3] = std::clamp(ds / h, -2.5, 2.5);
      z[i * fields + 5] =
          std::clamp(std::atan(wheelbase * (ref[i][2] - ref[i - 1][2]) /
                               (std::abs(ds) < 1e-9 ? 1e-9 : ds)),
                     -.75, .75);
    }
    for (int d = 0; d < 2; ++d) {
      double offset = d ? front_center : rear_center;
      z[i * fields + 7 + 2 * d] = ref[i][0] + offset * cos(ref[i][2]);
      z[i * fields + 8 + 2 * d] = ref[i][1] + offset * sin(ref[i][2]);
    }
  }
  return z;
}

void update_bounds(const Scene &scene, const std::vector<double> &previous,
                   std::vector<double> &lb, std::vector<double> &ub) {
  lb.assign(previous.size(), -1e20);
  ub.assign(previous.size(), 1e20);
  lb.back() = .1;
  ub.back() = nodes * .5;
  for (int i = 0; i < nodes; ++i) {
    for (int j = 3; j <= 6; ++j) {
      double bound = j == 3 ? 2.5 : j == 4 ? 1. : j == 5 ? .75 : .5;
      lb[i * fields + j] = -bound;
      ub[i * fields + j] = bound;
    }
    for (int d = 0; d < 2; ++d) {
      double offset = d ? front_center : rear_center;
      auto box = grow_box(
          scene, previous[i * fields] + offset * cos(previous[i * fields + 2]),
          previous[i * fields + 1] + offset * sin(previous[i * fields + 2]));
      int index = i * fields + 7 + 2 * d;
      lb[index] = box[0];
      ub[index] = box[1];
      lb[index + 1] = box[2];
      ub[index + 1] = box[3];
    }
  }
  for (int i : {0, nodes - 1}) {
    for (int j = 0; j < 2; ++j)
      lb[i * fields + j] = ub[i * fields + j] = scene.boundary[(i ? 3 : 0) + j];
    for (int j = 3; j <= 6; ++j)
      lb[i * fields + j] = ub[i * fields + j] = 0;
  }
  lb[2] = ub[2] = scene.boundary[2];
}

double validate(const Scene &scene, const std::vector<double> &z,
                const std::vector<double> &lb, const std::vector<double> &ub) {
  for (size_t i = 0; i < z.size(); ++i)
    if (!std::isfinite(z[i]) || z[i] < lb[i] - 1e-5 || z[i] > ub[i] + 1e-5)
      throw std::runtime_error("A hard bound is violated");
  // Test the actual covering discs against the obstacle triangles,
  // independently of NLP auxiliary centers.
  double minimum_clearance = 1e20, radius = std::hypot(4.689 / 4, 1.942 / 2);
  for (int i = 0; i < nodes; ++i)
    for (double offset : {rear_center, front_center}) {
      double x = z[i * fields] + offset * cos(z[i * fields + 2]),
             y = z[i * fields + 1] + offset * sin(z[i * fields + 2]);
      for (size_t o = 0; o < scene.obstacles.size(); o += 8) {
        bool positive = true, negative = true;
        double distance = 1e20;
        for (int e = 0; e < 3; ++e) {
          int n = (e + 1) % 3;
          double ax = scene.obstacles[o + 2 * e],
                 ay = scene.obstacles[o + 2 * e + 1],
                 dx = scene.obstacles[o + 2 * n] - ax,
                 dy = scene.obstacles[o + 2 * n + 1] - ay, px = x - ax,
                 py = y - ay, cross = dx * py - dy * px;
          positive &= cross >= -1e-10;
          negative &= cross <= 1e-10;
          double t =
              std::clamp((px * dx + py * dy) / (dx * dx + dy * dy), 0., 1.);
          distance = std::min(distance, std::hypot(px - t * dx, py - t * dy));
        }
        if (positive || negative)
          throw std::runtime_error(
              "Actual vehicle disc center lies in an obstacle");
        minimum_clearance = std::min(minimum_clearance, distance - radius);
      }
    }
  if (minimum_clearance < -1e-5)
    throw std::runtime_error("Actual covering-disc collision detected");
  return minimum_clearance;
}

int main(int argc, char **argv) {
  try {
    fs::path data = PARKING_DATA_DIR, output = "results";
    std::string linear_solver = "mumps";
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--data" && i + 1 < argc)
        data = fs::u8path(argv[++i]);
      else if (arg == "--output" && i + 1 < argc)
        output = fs::u8path(argv[++i]);
      else if (arg == "--linear-solver" && i + 1 < argc)
        linear_solver = argv[++i];
      else
        throw std::runtime_error("Usage: liom_demo [--data DIR] [--output DIR] "
                                 "[--linear-solver mumps]");
    }
    auto begin = std::chrono::steady_clock::now();
    Scene scene = load_scene(data);
    auto path = search(scene);
    double length;
    auto reference = resample(path, length);
    auto solution = initial_guess(reference, length);
    SX z = SX::sym("state", fields * nodes + 1),
       h = z(fields * nodes) / (nodes - 1), psi = 0, cost = z(fields * nodes);
    auto q = [&](int i, int j) { return z(i * fields + j); };
    auto sq = [](const SX &value) { return value * value; };
    for (int i = 0; i < nodes; ++i) {
      cost +=
          .025 * sq(q(i, 4)) + .00083983 * sq(q(i, 5)) + .0051 * sq(q(i, 6));
      if (i) {
        psi += sq(q(i, 0) - q(i - 1, 0) - h * q(i, 3) * cos(q(i, 2)));
        psi += sq(q(i, 1) - q(i - 1, 1) - h * q(i, 3) * sin(q(i, 2)));
        psi +=
            sq(q(i, 2) - q(i - 1, 2) - h * q(i, 3) * tan(q(i, 5)) / wheelbase);
        psi += sq(q(i, 3) - q(i - 1, 3) - h * q(i, 4));
        psi += sq(q(i, 5) - q(i - 1, 5) - h * q(i, 6));
        for (int d = 0; d < 2; ++d) {
          double offset = d ? front_center : rear_center;
          psi += sq(q(i, 7 + 2 * d) - q(i, 0) - offset * cos(q(i, 2)));
          psi += sq(q(i, 8 + 2 * d) - q(i, 1) - offset * sin(q(i, 2)));
        }
      }
    }
    psi += sq(sin(q(nodes - 1, 2)) - sin(scene.boundary[5])) +
           sq(cos(q(nodes - 1, 2)) - cos(scene.boundary[5]));
    casadi::Dict options;
    options["print_time"] = false;
    options["ipopt.print_level"] = 0;
    options["ipopt.sb"] = "yes";
    options["ipopt.tol"] = 1e-7;
    options["ipopt.max_iter"] = 6000;
    options["ipopt.max_cpu_time"] = 120.;
    options["ipopt.mu_strategy"] = "adaptive";
    options["ipopt.linear_solver"] = linear_solver;
    if (const char *hsl = std::getenv("HSL_LIBRARY"))
      options["ipopt.hsllib"] = hsl;
    auto solver = casadi::nlpsol(
        "liom", "ipopt", casadi::SXDict{{"x", z}, {"f", cost + 1e5 * psi}},
        options);
    casadi::Function metrics("metrics", {z}, {psi, cost});
    std::vector<double> lb, ub;
    bool accepted = false;
    int iterations = 0;
    double infeasibility = 1e20, objective = 0;
    for (int iteration = 1; iteration <= 5; ++iteration) {
      update_bounds(scene, solution, lb, ub);
      auto answer =
          solver(casadi::DMDict{{"x0", solution}, {"lbx", lb}, {"ubx", ub}});
      if (!bool(solver.stats().at("success")))
        throw std::runtime_error(
            solver.stats().at("return_status").to_string());
      solution = answer.at("x").nonzeros();
      auto values = metrics(std::vector<DM>{DM(solution)});
      infeasibility = double(values[0]);
      objective = double(values[1]);
      iterations = iteration;
      std::cout << "LIOM iteration " << iteration
                << ": infeasibility=" << infeasibility
                << ", objective=" << objective << '\n';
      if (infeasibility < 1e-4) {
        accepted = true;
        break;
      }
    }
    if (!accepted)
      throw std::runtime_error("LIOM reached its five-iteration limit before "
                               "meeting the paper's acceptance criterion");
    double clearance = validate(scene, solution, lb, ub);
    fs::create_directories(output);
    std::ofstream file(output / "trajectory.csv");
    file << std::setprecision(17) << "t,x,y,theta,v,a,phi,omega\n";
    for (int i = 0; i < nodes; ++i) {
      file << i * solution.back() / (nodes - 1);
      for (int j = 0; j < 7; ++j)
        file << ',' << solution[i * fields + j];
      file << '\n';
    }
    std::ofstream warm(output / "reference.csv");
    warm << "x,y,theta\n";
    for (auto p : reference)
      warm << p[0] << ',' << p[1] << ',' << p[2] << '\n';
    std::cout << "Case 1: Tf=" << solution.back() << " s, nodes=" << nodes
              << ", iterations=" << iterations
              << ", covering-disc clearance=" << clearance << " m, planning="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - begin)
                     .count()
              << " s\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
