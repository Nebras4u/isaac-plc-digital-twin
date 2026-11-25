#include "kuka_supervised_control/rt_executor.hpp"
#include "kuka_supervised_control/plc_bridge.hpp"
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>

namespace kuka_supervised_control
{
RtExecutor::~RtExecutor() { stop(); }

bool RtExecutor::configure(const Config & c)
{
  if (c.period_s < 0.005 || c.period_s > 0.100) { last_error_ = "period_s"; return false; }
  if (c.hb_timeout_s < 0.050 || c.hb_timeout_s > 1.0) { last_error_ = "hb_timeout_s"; return false; }
  if (c.cmd_stale_timeout_s < 0.050 || c.cmd_stale_timeout_s > 1.0) { last_error_ = "stale"; return false; }
  if (c.plc.recv_timeout_ms > static_cast<int>(c.hb_timeout_s * 500.0)) { last_error_ = "recv_timeout"; return false; }
  if (c.enable_rt) {
    if (c.rt_priority < 1 || c.rt_priority > 99) { last_error_ = "rt_priority"; return false; }
    if (c.rt_cpu_core < 0 || c.rt_cpu_core >= static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN))) { last_error_ = "cpu_core"; return false; }
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (!(c.soft_min_deg.at(i) < c.soft_max_deg.at(i))) { last_error_ = "limits"; return false; }
  }
  if (!bridge_) bridge_ = std::make_unique<PlcBridge>();
  if (!bridge_->configure(c.plc)) { last_error_ = "plc_cfg"; return false; }
  cfg_ = c;
  hb_timeout_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg_.hb_timeout_s));
  stale_timeout_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg_.cmd_stale_timeout_s));
  state_buffer_.reset(StateSnapshot{}); cmd_buffer_.reset(CommandSnapshot{});
  return true;
}

bool RtExecutor::start(double connect_timeout_s)
{
  if (running_.load(std::memory_order_acquire)) return false;
  have_hb_ = false; hb_was_fresh_ = false; cmd_was_stale_ = false; crc_streak_ = 0;
  last_good_ = StateSnapshot{}; state_buffer_.reset(StateSnapshot{}); cmd_buffer_.reset(CommandSnapshot{});
  running_.store(true, std::memory_order_release);
  thread_ = std::thread(&RtExecutor::run, this);
  if (connect_timeout_s > 0.0) {
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(connect_timeout_s));
    while (Clock::now() < deadline) {
      const auto snap = read_state();
      if (snap.valid && snap.hb_fresh) {
        if (cfg_.require_rt && diag_.rt_setup_failures.load(std::memory_order_relaxed) != 0) { last_error_ = "RT setup"; stop(); return false; }
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    last_error_ = "Connect timeout"; stop(); return false;
  }
  return true;
}

void RtExecutor::stop() { running_.store(false, std::memory_order_release); if (thread_.joinable()) thread_.join(); }
void RtExecutor::write_command(const CommandSnapshot & cmd) { cmd_buffer_.write(cmd); }
StateSnapshot RtExecutor::read_state() { return state_buffer_.read(); }

void RtExecutor::configure_rt_thread()
{
  if (!cfg_.enable_rt) return;
  uint8_t fails = 0; pthread_t self = pthread_self();
  sched_param sp{}; sp.sched_priority = cfg_.rt_priority;
  if (pthread_setschedparam(self, SCHED_FIFO, &sp) != 0) fails |= 1u;
  cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(cfg_.rt_cpu_core, &cs);
  if (pthread_setaffinity_np(self, sizeof(cs), &cs) != 0) fails |= 2u;
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) fails |= 4u;
  diag_.rt_setup_failures.store(fails, std::memory_order_relaxed);
}

bool RtExecutor::hb_is_fresh(Clock::time_point now) const { return have_hb_ && ((now - last_hb_change_) < hb_timeout_); }

void RtExecutor::invalidate_state()
{
  StateSnapshot s = last_good_; s.valid = false; s.hb_fresh = false; s.plc_axis_enable = false; state_buffer_.write(s);
}

void RtExecutor::run()
{
  configure_rt_thread();
  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(cfg_.period_s * 1e9));
  auto next_tick = Clock::now();
  while (running_.load(std::memory_order_acquire)) {
    const auto t0 = Clock::now(); cycle(t0); const auto t1 = Clock::now();
    const uint32_t us = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    diag_.last_cycle_us.store(us, std::memory_order_relaxed);
    uint32_t prev_max = diag_.max_cycle_us.load(std::memory_order_relaxed);
    while (us > prev_max && !diag_.max_cycle_us.compare_exchange_weak(prev_max, us)) {}
    next_tick += period_ns;
    if (t1 > next_tick) { diag_.overruns.fetch_add(1, std::memory_order_relaxed); next_tick = t1 + period_ns; }
    else { std::this_thread::sleep_until(next_tick); }
  }
  if (bridge_ && bridge_->connected()) {
    ProposalMsg safe; safe.seq = ++seq_; safe.hb_echo = last_hb_seen_; safe.flags = 0;
    std::array<uint8_t, kDbProposalSize> tx{}; safe.encode(tx.data()); bridge_->write_proposal(tx.data(), tx.size());
  }
  if (bridge_) bridge_->disconnect();
}

void RtExecutor::cycle(Clock::time_point now)
{
  if (!bridge_->connected()) {
    const int prev_reconn = bridge_->reconnect_count();
    auto r = bridge_->try_connect(now);
    if (!r.ok() && r.status == IoStatus::ConnectFailed) diag_.last_io_code.store(r.error_code, std::memory_order_relaxed);
    if (bridge_->reconnect_count() != prev_reconn) diag_.reconnects.fetch_add(1, std::memory_order_relaxed);
    if (!bridge_->connected()) { invalidate_state(); return; }
  }
  std::array<uint8_t, kDbFeedbackSize> rx{};
  auto rr = bridge_->read_feedback(rx.data(), rx.size());
  if (!rr.ok()) { diag_.io_errors.fetch_add(1, std::memory_order_relaxed); diag_.last_io_code.store(rr.error_code, std::memory_order_relaxed); invalidate_state(); return; }
  ingest_feedback(rx.data(), now); send_proposal(now);
}

void RtExecutor::ingest_feedback(const uint8_t * rx, Clock::time_point now)
{
  FeedbackMsg fb;
  if (!FeedbackMsg::decode(rx, fb)) {
    diag_.crc_errors.fetch_add(1, std::memory_order_relaxed);
    if (++crc_streak_ >= cfg_.max_consecutive_crc_errors) invalidate_state();
    return;
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(fb.actual_pos_deg.at(i)) || !std::isfinite(fb.actual_vel_deg_s.at(i)) || !std::isfinite(fb.max_step_deg.at(i))) {
      diag_.bad_values.fetch_add(1, std::memory_order_relaxed); invalidate_state(); return;
    }
  }
  if (!std::isfinite(fb.max_vel_deg_s)) { diag_.bad_values.fetch_add(1, std::memory_order_relaxed); invalidate_state(); return; }
  crc_streak_ = 0;
  if (!have_hb_ || fb.plc_heartbeat != last_hb_seen_) { last_hb_seen_ = fb.plc_heartbeat; last_hb_change_ = now; have_hb_ = true; }
  const bool fresh = hb_is_fresh(now);
  if (hb_was_fresh_ && !fresh) diag_.hb_timeouts.fetch_add(1, std::memory_order_relaxed);
  hb_was_fresh_ = fresh;
  diag_.last_plc_state.store(fb.plc_state, std::memory_order_relaxed); diag_.last_decision.store(fb.decision, std::memory_order_relaxed); diag_.last_reject.store(fb.reject_reason, std::memory_order_relaxed);

  StateSnapshot snap;
  for (size_t i = 0; i < kNumJoints; ++i) {
    snap.actual_pos_rad.at(i) = fb.actual_pos_deg.at(i) * kDegToRad;
    snap.actual_vel_rad_s.at(i) = fb.actual_vel_deg_s.at(i) * kDegToRad;
    snap.max_step_rad.at(i) = fb.max_step_deg.at(i) * kDegToRad;
  }
  snap.max_vel_rad_s = static_cast<double>(fb.max_vel_deg_s) * kDegToRad;
  snap.plc_state = fb.plc_state; snap.decision = fb.decision; snap.reject_reason = fb.reject_reason;
  snap.ack_proposal_id = fb.ack_proposal_id; snap.ack_seq = fb.ack_seq; snap.plc_hb = fb.plc_heartbeat;
  snap.plc_axis_enable = (fb.plc_flags & PlcFlagAxisEnable) != 0; snap.plc_estop = (fb.plc_flags & PlcFlagEstopActive) != 0;
  snap.hb_fresh = fresh; snap.rx_steady = now; snap.hb_change_steady = last_hb_change_; snap.valid = true;
  last_good_ = snap; state_buffer_.write(snap);
}

void RtExecutor::send_proposal(Clock::time_point now)
{
  ProposalMsg prop; prop.seq = ++seq_; prop.hb_echo = last_hb_seen_;
  const CommandSnapshot & cmd = cmd_buffer_.read();
  if (cmd.valid) {
    const bool stale = (now - cmd.written_at) > stale_timeout_;
    if (stale && !cmd_was_stale_) diag_.stale_cmds.fetch_add(1, std::memory_order_relaxed);
    cmd_was_stale_ = stale;
    prop.proposal_id = cmd.proposal_id;
    prop.state_request = stale ? static_cast<uint16_t>(StateRequest::Idle) : cmd.state_request;
    prop.flags = stale ? 0 : cmd.flags;
    const bool have_state = last_good_.valid;
    for (size_t i = 0; i < kNumJoints; ++i) {
      double deg = cmd.proposed_pos_rad.at(i) * kRadToDeg;
      if (!std::isfinite(deg)) deg = have_state ? (last_good_.actual_pos_rad.at(i) * kRadToDeg) : 0.0;
      if (have_state && last_good_.max_step_rad.at(i) > 0.0) {
        const double actual_deg = last_good_.actual_pos_rad.at(i) * kRadToDeg;
        const double max_step_deg = last_good_.max_step_rad.at(i) * kRadToDeg;
        if (std::abs(deg - actual_deg) > max_step_deg) { deg = actual_deg + std::copysign(max_step_deg, deg - actual_deg); diag_.cmd_clamps.fetch_add(1, std::memory_order_relaxed); }
      }
      deg = std::clamp(deg, cfg_.soft_min_deg.at(i), cfg_.soft_max_deg.at(i));
      prop.proposed_pos_deg.at(i) = deg;
    }
  }
  std::array<uint8_t, kDbProposalSize> tx{}; prop.encode(tx.data());
  auto wr = bridge_->write_proposal(tx.data(), tx.size());
  if (!wr.ok()) { diag_.io_errors.fetch_add(1, std::memory_order_relaxed); diag_.last_io_code.store(wr.error_code, std::memory_order_relaxed); }
}
}  // namespace kuka_supervised_control
