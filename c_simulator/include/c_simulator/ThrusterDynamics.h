#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>
namespace c_simulator {
struct ThrusterParameters {
  double delay = .1, rise = .08, fall = .06, slew = 300, deadband = 0;
  double forwardLimit = 28, reverseLimit = 28, forwardScale = 1,
         reverseScale = 1, efficiency = 1;
};
// Simulation-time actuator dynamics, independent of ROS scheduling and wall
// time.
class ThrusterDynamics {
public:
  void configure(const std::vector<ThrusterParameters> &parameters,
                 double timeout = .5);
  void command(const Eigen::VectorXd &force);
  void advance(double dt);
  void stop(); // Cancel queued commands immediately; propellers coast down.
  void reset();
  double time() const { return time_; }
  const Eigen::VectorXd &forces() const { return forces_; }

private:
  struct Command {
    double time;
    double force;
  };
  std::vector<ThrusterParameters> parameters_;
  std::vector<std::deque<Command>> queues_;
  Eigen::VectorXd targets_, forces_;
  double time_ = 0, timeout_ = .5, lastCommand_ = -1e9;
  void evolve(int i, double dt);
};
} // namespace c_simulator
