#ifndef KUKA_SUPERVISED_CONTROL__RT_EXECUTOR_HPP_
#define KUKA_SUPERVISED_CONTROL__RT_EXECUTOR_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "kuka_supervised_control/plc_transport.hpp"
#include "kuka_supervised_control/spsc_triple_buffer.hpp"
#include "kuka_supervised_control/supervisor_protocol.hpp"

namespace kuka_supervised_control
{

// Published by the executor thread -> consumed by the controller-manager
// thread in read(). Units = radians.
struct StateSnapshot {
  std::array<double, kNumJoints> actual_pos_rad{};
  std::array<double, kNumJoints> actual_vel_rad_s{};
  std::array<double, kNumJoints> max_step_rad{};   // per-cycle budget from PLC
  double   max_vel_rad_s{0.0};
  uint16_t plc_state{0};
  uint16_t decision{0};
  uint16_t reject_reason{0};
  uint16_t ack_proposal_id{0};
  uint32_t ack_seq{0};
  int32_t  plc_hb{0};
  bool     plc_axis_enable{false};
  bool     plc_estop{true};      // safe default
  bool     hb_fresh{false};
  Clock::time_point rx_steady{};
  Clock::time_point hb_change_steady{};
  bool     valid{false};
};

// Written by the controller-manager thread in write() -> consumed by the
// executor thread. Units = radians.
struct CommandSnapshot {
  std::array<double, kNumJoints> proposed_pos_rad{};
  uint16_t proposal_id{0};
  uint16_t state_request{0};   // StateRequest
  uint8_t  flags{0};           // RosFlags
  Clock::time_point written_at{};
  bool     valid{false};       // false = "no command yet"
};

// Atomic counters, readable from any thread. No strings, no logging in RT.
struct RtDiagnostics {
  std::atomic<uint32_t> io_errors{0};
  std::atomic<uint32_t> reconnects{0};
  std::atomic<uint32_t> hb_timeouts{0};     // counted on the fresh->stale edge
  std::atomic<uint32_t> crc_errors{0};
  std::atomic<uint32_t> bad_values{0};      // NaN/inf in a CRC-valid frame
  std::atomic<uint32_t> overruns{0};
  std::atomic<uint32_t> stale_cmds{0};      // counted on the fresh->stale edge
  std::atomic<uint32_t> cmd_clamps{0};
  std::atomic<uint32_t> last_cycle_us{0};
  std::atomic<uint32_t> max_cycle_us{0};
  std::atomic<int>      last_io_code{0};
  std::atomic<uint16_t> last_plc_state{0};
  std::atomic<uint16_t> last_decision{0};
  std::atomic<uint16_t> last_reject{0};
  std::atomic<uint8_t>  rt_setup_failures{0};  // bit0 sched, bit1 affinity, bit2 mlock
};

class RtExecutor
{
public:
  struct Config {
    bool   enable_rt{false};
    bool   require_rt{false};          // fail start() if RT setup failed
    int    rt_priority{80};
    int    rt_cpu_core{2};
    double period_s{0.020};

    double hb_timeout_s{0.100};
    double cmd_stale_timeout_s{0.100};
    int    max_consecutive_crc_errors{3};

    // Mandatory limits in degrees.
    std::array<double, kNumJoints> soft_min_deg{};
    std::array<double, kNumJoints> soft_max_deg{};

    PlcConfig plc;
  };

  RtExecutor() = default;
  ~RtExecutor();
  RtExecutor(const RtExecutor &) = delete;
  RtExecutor & operator=(const RtExecutor &) = delete;

  // Optional: inject another transport (OPC UA, mock). Call before configure().
  void set_transport(std::unique_ptr<PlcTransport> t) { bridge_ = std::move(t); }

  bool configure(const Config & cfg);

  // Blocks up to connect_timeout_s waiting for the first fresh PLC feedback.
  bool start(double connect_timeout_s);
  void stop();

  // Controller-manager thread ONLY (single producer of the command buffer).
  void write_command(const CommandSnapshot & cmd);
  // Controller-manager thread ONLY (single consumer of the state buffer).
  StateSnapshot read_state();

  const RtDiagnostics & diagnostics() const noexcept { return diag_; }
  const std::string & last_error() const noexcept { return last_error_; }

private:
  void run();
  void cycle(Clock::time_point now);
  void ingest_feedback(const uint8_t * rx, Clock::time_point now);
  void send_proposal(Clock::time_point now);
  void invalidate_state();
  bool hb_is_fresh(Clock::time_point now) const;
  void configure_rt_thread();

  Config cfg_{};
  std::unique_ptr<PlcTransport> bridge_;
  RtDiagnostics diag_{};
  std::string last_error_;

  Clock::duration hb_timeout_{};
  Clock::duration stale_timeout_{};

  // Executor-thread-private state.
  int32_t  last_hb_seen_{0};
  bool     have_hb_{false};
  bool     hb_was_fresh_{false};
  bool     cmd_was_stale_{false};
  int      crc_streak_{0};
  Clock::time_point last_hb_change_{};
  uint32_t seq_{0};
  StateSnapshot last_good_{};     // last CRC-valid, finite feedback

  SpscTripleBuffer<StateSnapshot>   state_buffer_;  // executor -> CM
  SpscTripleBuffer<CommandSnapshot> cmd_buffer_;    // CM -> executor

  std::thread       thread_;
  std::atomic<bool> running_{false};
};

}  // namespace kuka_supervised_control

#endif
