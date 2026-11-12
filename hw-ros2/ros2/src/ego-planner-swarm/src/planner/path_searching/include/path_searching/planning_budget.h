#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace ego_planner
{
// One immutable, monotonic deadline shared by all attempts in an FSM cycle.
// Cooperative cancellation: individual non-interruptible operations may overrun.
class PlanningBudget
{
public:
  using Clock = std::chrono::steady_clock;
  using Ptr = std::shared_ptr<const PlanningBudget>;

  explicit PlanningBudget(double seconds)
  {
    if (!std::isfinite(seconds) || seconds <= 0.0)
      throw std::invalid_argument("Planning budget must be finite and positive");
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(seconds));
  }

  bool expired() const { return Clock::now() >= deadline_; }
  Clock::time_point deadline() const { return deadline_; }

private:
  Clock::time_point deadline_;
};
}  // namespace ego_planner
