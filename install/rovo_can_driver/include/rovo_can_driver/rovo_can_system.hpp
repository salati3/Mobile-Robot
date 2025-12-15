#ifndef ROVO_CAN_DRIVER__ROVO_CAN_SYSTEM_HPP_
#define ROVO_CAN_DRIVER__ROVO_CAN_SYSTEM_HPP_

#include <array>
#include <string>
#include <cstdint>
#include <vector>
#include <memory>

#include <linux/can.h>

#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

namespace rovo_can_driver
{

// --- Constants for CAN IDs ---
static constexpr uint32_t CAN_ID_EC_TPDO1 = 0x18A; // Control & Mode
static constexpr uint32_t CAN_ID_EC_TPDO2 = 0x28A; // Speed & Stage
static constexpr uint32_t CAN_ID_EC_PW_TX = 0x38A; // Password
static constexpr uint32_t CAN_ID_EC_RPDO1 = 0x20A; // Status Feedback
static constexpr uint32_t CAN_ID_EC_RPDO2 = 0x30A; // Speed & Battery Feedback
static constexpr uint32_t CAN_ID_ERR_EXT  = 0x40A; // External Error Message

// --- State Machine for Activation ---
enum class ExternalControlPhase
{
  WAITING_FOR_READY = 0,      // Wait for MainState == 2
  REQUEST_STARTUP   = 1,      // Request UnlockState=1
  REQUEST_ACTIVE    = 2,      // Request UnlockState=2 + Password
  WAITING_FOR_DRIVE_MODE = 3, // Wait for ActualDriveMode == 2
  ACTIVE            = 4       // Fully Active, sending commands
};

class RovoCanSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RovoCanSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // CAN Low-Level
  bool open_can_interface();
  void close_can_interface();
  void send_blocking_command(const struct can_frame & frame);

  // Protocol Handling
  void process_can_frame(const struct can_frame & frame);
  void send_ec_tpdo1(uint8_t requested_unlock_state, uint8_t requested_drive_mode);
  void send_speed_frame();
  void send_password_frame();
  
private:
  rclcpp::Logger logger_{rclcpp::get_logger("rovo_can_system")};

  // Joint Data
  std::array<double, 2> hw_positions_{};   // [rad]
  std::array<double, 2> hw_velocities_{};  // [rad/s]
  std::array<double, 2> hw_commands_{};    // [rad/s] desired

  // Configuration Parameters
  std::string can_interface_name_{"can0"};
  double wheel_separation_{0.8};   // [m]
  double wheel_radius_{0.12};      // [m]
  double gear_ratio_{16.0};        
  double max_motor_rpm_{2000.0};   
  uint8_t driving_stage_{1};       
  bool enable_external_hv_{false}; // Configurable HV parameter

  // Feedback State
  double battery_voltage_{0.0};    // [V]
  double battery_current_{0.0};    // [A]
  uint8_t main_state_{0};          // 2 = Active
  uint8_t unlock_state_{0};        // 2 = Ext Control Active
  uint8_t drive_mode_{0};          // 2 = RPM Control
  uint8_t max_driving_stage_{0};
  
  // Watchdog
  rclcpp::Time last_write_time_;

  // Internal State Machine
  ExternalControlPhase control_phase_{ExternalControlPhase::WAITING_FOR_READY};
  uint8_t tx_alive_counter_{0};
  bool password_sent_{false};

  // Authentication
  std::array<uint8_t, 6> password_{{0x4D, 0x41, 0x54, 0x54, 0x52, 0x4F}}; // "MATTRO"

  // CAN Socket
  int  can_socket_{-1};
  bool can_connected_{false};
};

}  // namespace rovo_can_driver

#endif  // ROVO_CAN_DRIVER__ROVO_CAN_SYSTEM_HPP_