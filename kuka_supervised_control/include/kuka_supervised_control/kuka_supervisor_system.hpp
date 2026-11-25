#ifndef KUKA_SUPERVISED_CONTROL__KUKA_SUPERVISOR_SYSTEM_HPP_
#define KUKA_SUPERVISED_CONTROL__KUKA_SUPERVISOR_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "kuka_supervised_control/rt_executor.hpp"
#include "kuka_supervised_control/supervisor_protocol.hpp"

namespace kuka_supervised_control
{

class KukaSupervisorSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaSupervisorSystem)

  ~KukaSupervisorSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & p) override;
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void start_diag_thread();
  void stop_diag_thread();
  void diag_loop();

  RtExecutor rt_;
  std::vector<std::string> joint_names_;

  // Hardware interfaces (accessed only from the controller-manager thread).
  std::array<double, kNumJoints> hw_positions_rad_{};
  std::array<double, kNumJoints> hw_velocities_rad_s_{};
  std::array<double, kNumJoints> hw_commands_rad_{};
  double hw_axis_enable_{0.0};   // SAFE DEFAULT: must be explicitly set to 1.0
  double hw_estop_{0.0};
  double hw_reset_{0.0};         // level: >0.5 requests fault reset

  // Read-only "safety" state interfaces.
  double hw_comm_ok_{0.0};
  double hw_plc_state_{0.0};
  double hw_plc_estop_{1.0};
  double hw_plc_axis_enable_{0.0};

  std::array<double, kNumJoints> last_proposed_rad_{};
  uint16_t proposal_id_{0};

  // Communication supervision.
  bool   comm_ok_{false};
  bool   bad_since_valid_{false};
  Clock::time_point bad_since_{};
  double comm_loss_tolerance_s_{0.200};
  bool   fail_on_comm_loss_{true};

  // Diagnostics logging lives OFF the real-time thread.
  std::thread       diag_thread_;
  std::atomic<bool> diag_run_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("KukaSupervisorSystem")};
};

}  // namespace kuka_supervised_control

#endif
