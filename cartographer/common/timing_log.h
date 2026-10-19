// Diagnostics only: wall-clock timing of the pose-graph background work, logged as "CARTO_TIMING <kind> ..." INFO lines
// when the environment variable CARTO_TIMING_LOG is set (2026-10-07, to find where the live correction delay's tail
// comes from). Off by default: no cost beyond one getenv.
#ifndef CARTOGRAPHER_COMMON_TIMING_LOG_H_
#define CARTOGRAPHER_COMMON_TIMING_LOG_H_
#include <chrono>
#include <cstdlib>
namespace cartographer {
namespace common {
inline bool TimingLogEnabled() {
  static const bool enabled = std::getenv("CARTO_TIMING_LOG") != nullptr;
  return enabled;
}
inline double WallSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace common
}  // namespace cartographer
#endif
