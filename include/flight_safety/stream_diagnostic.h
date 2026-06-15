#ifndef FLIGHT_SAFETY_STREAM_DIAGNOSTIC_H
#define FLIGHT_SAFETY_STREAM_DIAGNOSTIC_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/time.h>
#include <cmath>
#include <mutex>
#include <string>

// Reusable in-node self-report for a data stream: rate, dropout, and (optional)
// freeze. Mirrors mavros HeartbeatStatus (windowed frequency) and adds a value-freeze
// check for streams whose payload can go stale while still ticking (e.g. a mocap pose
// that keeps republishing the last sample after the tracker loses the body).
//
// A node #includes this and ticks it from inside (so the report dies with the node;
// the external flight_safety watchdog catches that). One implementation, reused by
// vrpn_client_ros / fast-livo / ... — add `flight_safety` to the consumer's CMakeLists.
//
//   flight_safety::StreamDiagnostic diag("pose", {/*min_rate*/ 50.0, 0.0, /*freeze_window*/ 0.5});
//   updater.add(diag);
//   diag.tick(stamp_ok);                 // liveness / rate
//   diag.tick(x, y, z);                  // + freeze on a 3D payload
//   updater.update();                    // periodically
namespace flight_safety
{

struct StreamDiagnosticConfig
{
  double min_rate     = 0.0;    // Hz; below -> WARN. 0 disables.
  double max_rate     = 0.0;    // Hz; above -> WARN. 0 disables.
  double freeze_window = 0.0;   // s; payload unchanged longer than this -> ERROR. 0 disables.
  double freeze_eps   = 1e-6;   // value deadband for "changed".
  double tolerance    = 0.1;    // rate band tolerance (matches mavros HeartbeatStatus).
};

class StreamDiagnostic : public diagnostic_updater::DiagnosticTask
{
public:
  StreamDiagnostic(const std::string &name, const StreamDiagnosticConfig &cfg)
    : diagnostic_updater::DiagnosticTask(name), cfg_(cfg)
  {
    const ros::Time now = ros::Time::now();
    last_run_ = now;
    last_change_ = now;
  }

  void tick() { tick_impl(false, 0.0, 0.0, 0.0); }                       // liveness / rate
  void tick(double x, double y, double z) { tick_impl(true, x, y, z); } // + freeze on 3D value

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time now = ros::Time::now();
    const double window = (now - last_run_).toSec();
    const int events = count_ - last_count_;
    const double freq = (window > 0.0) ? events / window : 0.0;
    const double freeze_age = (now - last_change_).toSec();
    last_run_ = now;
    last_count_ = count_;

    if (events == 0)
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "no data (dropout)");
    else if (cfg_.freeze_window > 0.0 && have_pos_ && freeze_age > cfg_.freeze_window)
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "frozen (%.2fs)", freeze_age);
    else if (cfg_.min_rate > 0.0 && freq < cfg_.min_rate * (1.0 - cfg_.tolerance))
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "rate low (%.1f Hz)", freq);
    else if (cfg_.max_rate > 0.0 && freq > cfg_.max_rate * (1.0 + cfg_.tolerance))
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "rate high (%.1f Hz)", freq);
    else
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "ok (%.1f Hz)", freq);

    stat.addf("rate_hz", "%.1f", freq);
    stat.addf("count", "%d", count_);
    if (cfg_.freeze_window > 0.0)
      stat.addf("freeze_s", "%.2f", freeze_age);
  }

private:
  void tick_impl(bool has_pos, double x, double y, double z)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++count_;
    if (has_pos)
    {
      if (!have_pos_ ||
          std::fabs(x - px_) > cfg_.freeze_eps ||
          std::fabs(y - py_) > cfg_.freeze_eps ||
          std::fabs(z - pz_) > cfg_.freeze_eps)
        last_change_ = ros::Time::now();
      px_ = x; py_ = y; pz_ = z; have_pos_ = true;
    }
  }

  StreamDiagnosticConfig cfg_;
  std::mutex mutex_;
  int count_ = 0, last_count_ = 0;
  ros::Time last_run_, last_change_;
  bool have_pos_ = false;
  double px_ = 0.0, py_ = 0.0, pz_ = 0.0;
};

}  // namespace flight_safety
#endif  // FLIGHT_SAFETY_STREAM_DIAGNOSTIC_H
