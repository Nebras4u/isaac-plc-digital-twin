#include "kuka_supervised_control/kuka_supervisor_system.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace kuka_supervised_control
{
namespace
{
template <typename T>
T get_param(const hardware_interface::HardwareInfo & info, const std::string & k, T def)
{
  auto it = info.hardware_parameters.find(k);
  if (it == info.hardware_parameters.end()) return def;
  std::istringstream ss(it->second);
  T v; ss >> v;
  return ss.fail() ? def : v;
}
std::array<double, kNumJoints> get_array_param(const hardware_interface::HardwareInfo & info, const std::string & k)
{
  std::array<double, kNumJoints> a{};
  auto it = info.hardware_parameters.find(k);
  if (it == info.hardware_parameters.end()) return a;
  std::istringstream ss(it->second);
  std::string tok; size_t i = 0;
  while (std::getline(ss, tok, ',') && i < kNumJoints) a.at(i++) = std::stod(tok);
  return a;
}
}  // namespace

KukaSupervisorSystem::~KukaSupervisorSystem() { stop_diag_thread(); rt_.stop(); }

hardware_interface::CallbackReturn KukaSupervisorSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & p)
{
  if (SystemInterface::on_init(p) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  if (info_.joints.size() != kNumJoints) return hardware_interface::CallbackReturn::ERROR;
  joint_names_.clear();
  for (const auto & j : info_.joints) joint_names_.push_back(j.name);

  comm_loss_tolerance_s_ = get_param<double>(info_, "comm_loss_tolerance_s", 0.200);
  fail_on_comm_loss_     = get_param<bool>(info_, "fail_on_comm_loss", false);

  RtExecutor::Config cfg;
  cfg.enable_rt           = get_param<bool>(info_, "enable_rt", false);
  cfg.require_rt          = get_param<bool>(info_, "require_rt", false);
  cfg.rt_priority         = get_param<int>(info_, "rt_priority", 80);
  cfg.rt_cpu_core         = get_param<int>(info_, "rt_cpu_core", 2);
  cfg.period_s            = get_param<double>(info_, "period_s", 0.020);
  cfg.hb_timeout_s        = get_param<double>(info_, "hb_timeout_s", 0.100);
  cfg.cmd_stale_timeout_s = get_param<double>(info_, "cmd_stale_timeout_s", 0.100);
  cfg.soft_min_deg        = get_array_param(info_, "soft_min_deg");
  cfg.soft_max_deg        = get_array_param(info_, "soft_max_deg");

  cfg.plc.ip                 = get_param<std::string>(info_, "plc_ip", "192.168.99.56");
  cfg.plc.rack               = get_param<int>(info_, "plc_rack", 0);
  cfg.plc.slot               = get_param<int>(info_, "plc_slot", 1);
  cfg.plc.db_proposal        = get_param<int>(info_, "db_proposal", 100);
  cfg.plc.db_feedback        = get_param<int>(info_, "db_feedback", 101);
  cfg.plc.connect_timeout_ms = get_param<int>(info_, "connect_timeout_ms", 300);
  cfg.plc.recv_timeout_ms    = get_param<int>(info_, "recv_timeout_ms", 40);
  cfg.plc.send_timeout_ms    = get_param<int>(info_, "send_timeout_ms", 40);
  cfg.plc.reconnect_min_ms   = get_param<int>(info_, "reconnect_min_ms", 100);
  cfg.plc.reconnect_max_ms   = get_param<int>(info_, "reconnect_max_ms", 2000);
  cfg.plc.proposal_size      = kDbProposalSize;
  cfg.plc.feedback_size      = kDbFeedbackSize;

  if (!rt_.configure(cfg)) {
    RCLCPP_ERROR(logger_, "RtExecutor::configure failed: %s", rt_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaSupervisorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < kNumJoints; ++i) {
    si.emplace_back(joint_names_.at(i), hardware_interface::HW_IF_POSITION, &hw_positions_rad_.at(i));
    si.emplace_back(joint_names_.at(i), hardware_interface::HW_IF_VELOCITY, &hw_velocities_rad_s_.at(i));
  }
  si.emplace_back("safety", "comm_ok", &hw_comm_ok_);
  si.emplace_back("safety", "plc_state", &hw_plc_state_);
  si.emplace_back("safety", "plc_estop", &hw_plc_estop_);
  si.emplace_back("safety", "plc_axis_enable", &hw_plc_axis_enable_);
  return si;
}

std::vector<hardware_interface::CommandInterface> KukaSupervisorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < kNumJoints; ++i) {
    ci.emplace_back(joint_names_.at(i), hardware_interface::HW_IF_POSITION, &hw_commands_rad_.at(i));
  }
  ci.emplace_back("safety", "axis_enable", &hw_axis_enable_);
  ci.emplace_back("safety", "estop", &hw_estop_);
  ci.emplace_back("safety", "reset", &hw_reset_);
  return ci;
}

hardware_interface::CallbackReturn KukaSupervisorSystem::on_activate(const rclcpp_lifecycle::State &)
{
  const double connect_timeout_s = get_param<double>(info_, "connect_timeout_s", 3.0);
  if (!rt_.start(connect_timeout_s)) {
    RCLCPP_ERROR(logger_, "Failed to start RtExecutor: %s", rt_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto snap = rt_.read_state();
  for (size_t i = 0; i < kNumJoints; ++i) {
    hw_positions_rad_.at(i) = snap.actual_pos_rad.at(i);
    hw_velocities_rad_s_.at(i) = snap.actual_vel_rad_s.at(i);
    hw_commands_rad_.at(i) = snap.actual_pos_rad.at(i);
  }
  last_proposed_rad_ = hw_commands_rad_;
  hw_axis_enable_ = 0.0; hw_estop_ = 0.0; hw_reset_ = 0.0;
  comm_ok_ = true; bad_since_valid_ = false;
  start_diag_thread();
  RCLCPP_INFO(logger_, "Activated v2. PLC state=%s hb=%d", to_string(PlcState(snap.plc_state)), snap.plc_hb);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KukaSupervisorSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_diag_thread();
  CommandSnapshot safe; safe.flags = 0; safe.valid = false; safe.written_at = Clock::now();
  rt_.write_command(safe);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  rt_.stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KukaSupervisorSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto snap = rt_.read_state();
  const auto now = Clock::now();
  if (!snap.valid || !snap.hb_fresh) {
    comm_ok_ = false; hw_comm_ok_ = 0.0; hw_plc_axis_enable_ = 0.0; hw_axis_enable_ = 0.0;
    if (!bad_since_valid_) { bad_since_ = now; bad_since_valid_ = true; }
    else if (fail_on_comm_loss_ && std::chrono::duration<double>(now - bad_since_).count() > comm_loss_tolerance_s_) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  comm_ok_ = true; bad_since_valid_ = false;
  for (size_t i = 0; i < kNumJoints; ++i) {
    hw_positions_rad_.at(i) = snap.actual_pos_rad.at(i);
    hw_velocities_rad_s_.at(i) = snap.actual_vel_rad_s.at(i);
  }
  hw_comm_ok_ = 1.0; hw_plc_state_ = static_cast<double>(snap.plc_state);
  hw_plc_estop_ = snap.plc_estop ? 1.0 : 0.0; hw_plc_axis_enable_ = snap.plc_axis_enable ? 1.0 : 0.0;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaSupervisorSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  CommandSnapshot cmd; cmd.written_at = Clock::now();
  if (!comm_ok_) { cmd.valid = false; rt_.write_command(cmd); return hardware_interface::return_type::OK; }
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(hw_commands_rad_.at(i))) { cmd.valid = false; rt_.write_command(cmd); return hardware_interface::return_type::OK; }
  }
  bool changed = false;
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (std::abs(hw_commands_rad_.at(i) - last_proposed_rad_.at(i)) * kRadToDeg > 0.01) { changed = true; break; }
  }
  if (changed) { ++proposal_id_; last_proposed_rad_ = hw_commands_rad_; }
  cmd.proposed_pos_rad = hw_commands_rad_; cmd.proposal_id = proposal_id_; cmd.flags = RosFlagProposalValid;
  if (hw_estop_ > 0.5) { cmd.flags |= RosFlagRequestEstop; cmd.state_request = static_cast<uint16_t>(StateRequest::Estop); }
  else if (hw_reset_ > 0.5) { cmd.state_request = static_cast<uint16_t>(StateRequest::Reset); }
  else if (hw_axis_enable_ > 0.5) { cmd.flags |= RosFlagRequestEnable; cmd.state_request = static_cast<uint16_t>(StateRequest::Run); }
  else { cmd.state_request = static_cast<uint16_t>(StateRequest::Idle); }
  cmd.valid = true; rt_.write_command(cmd);
  return hardware_interface::return_type::OK;
}

void KukaSupervisorSystem::start_diag_thread() { if (!diag_run_.exchange(true)) diag_thread_ = std::thread(&KukaSupervisorSystem::diag_loop, this); }
void KukaSupervisorSystem::stop_diag_thread() { diag_run_.store(false); if (diag_thread_.joinable()) diag_thread_.join(); }

void KukaSupervisorSystem::diag_loop()
{
  uint32_t l_io = 0, l_crc = 0, l_st = 0, l_hb = 0;
  while (diag_run_.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (!diag_run_.load()) break;
    const auto & d = rt_.diagnostics();
    const uint32_t io = d.io_errors.load(), crc = d.crc_errors.load(), st = d.stale_cmds.load(), hb = d.hb_timeouts.load();
    if (io != l_io || crc != l_crc || st != l_st || hb != l_hb) {
      std::array<char, 64> b{}; reject_reason_strings(d.last_reject.load(), b.data(), b.size());
      RCLCPP_WARN(logger_, "diag: io=%u crc=%u stale=%u hb_to=%u | state=%s dec=%s rej=%s",
        io, crc, st, hb, to_string(PlcState(d.last_plc_state.load())), to_string(Decision(d.last_decision.load())), b.data());
      l_io = io; l_crc = crc; l_st = st; l_hb = hb;
    }
  }
}
}  // namespace kuka_supervised_control
PLUGINLIB_EXPORT_CLASS(kuka_supervised_control::KukaSupervisorSystem, hardware_interface::SystemInterface)
