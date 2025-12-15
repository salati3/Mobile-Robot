#include "rovo_can_driver/rovo_can_system.hpp"

#include <chrono>
#include <cstring>
#include <cmath>
#include <limits>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>

#include "pluginlib/class_list_macros.hpp"

namespace rovo_can_driver
{

using hardware_interface::CallbackReturn;
using hardware_interface::HardwareComponentInterfaceParams;
using hardware_interface::return_type;

// ---------------- on_init ----------------

CallbackReturn RovoCanSystem::on_init(const HardwareComponentInterfaceParams & params)
{
  logger_ = get_logger();
  const auto & info = params.hardware_info;

  RCLCPP_INFO(logger_, "Initializing RovoCanSystem...");

  if (info.joints.size() != 2) {
    RCLCPP_FATAL(logger_, "Expected 2 joints, got %zu", info.joints.size());
    return CallbackReturn::ERROR;
  }

  // Parameter Parsing with Defaults
  for (const auto & kv : info.hardware_parameters) {
    const auto & name = kv.first;
    const auto & val  = kv.second;

    try {
      if (name == "can_interface") can_interface_name_ = val;
      else if (name == "wheel_separation") wheel_separation_ = std::stod(val);
      else if (name == "wheel_radius") wheel_radius_ = std::stod(val);
      else if (name == "gear_ratio") gear_ratio_ = std::stod(val);
      else if (name == "max_motor_rpm") max_motor_rpm_ = std::stod(val);
      else if (name == "driving_stage") driving_stage_ = static_cast<uint8_t>(std::stoi(val));
      else if (name == "enable_external_hv") enable_external_hv_ = (val == "true" || val == "1");
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Param '%s' invalid: %s. Using default.", name.c_str(), e.what());
    }
  }

  // Initialize State
  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_commands_.fill(0.0);
  
  RCLCPP_INFO(
    logger_, 
    "ROVO Driver Ready. CAN: %s, ExtHV: %s, Stage: %d", 
    can_interface_name_.c_str(), 
    enable_external_hv_ ? "ON" : "OFF", 
    driving_stage_);

  return CallbackReturn::SUCCESS;
}

// ---------------- Export Interfaces ----------------

std::vector<hardware_interface::StateInterface> RovoCanSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < 2; ++i) {
    // Standard Joint State Interfaces
    interfaces.emplace_back(get_hardware_info().joints[i].name, "position", &hw_positions_[i]);
    interfaces.emplace_back(get_hardware_info().joints[i].name, "velocity", &hw_velocities_[i]);
  }
  
  // Expose Battery Data as Sensor Interfaces
  // These can be read by other nodes or published by robot_state_publisher
  interfaces.emplace_back("rovo_battery", "voltage", &battery_voltage_);
  interfaces.emplace_back("rovo_battery", "current", &battery_current_);
  
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RovoCanSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < 2; ++i) {
    interfaces.emplace_back(get_hardware_info().joints[i].name, "velocity", &hw_commands_[i]);
  }
  return interfaces;
}

// ---------------- Lifecycle ----------------

CallbackReturn RovoCanSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RovoCanSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_can_interface()) return CallbackReturn::ERROR;

  // Reset state on activation
  hw_commands_.fill(0.0);
  tx_alive_counter_ = 0;
  password_sent_    = false;
  control_phase_    = ExternalControlPhase::WAITING_FOR_READY;
  last_write_time_  = rclcpp::Clock().now();

  RCLCPP_INFO(logger_, "Driver Activated. Waiting for ROVO VCU...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RovoCanSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (can_connected_) {
    // Send Safe Shutdown Command
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id  = CAN_ID_EC_TPDO1;
    frame.can_dlc = 3;
    frame.data[0] = 0; // UnlockState=0 (Idle)
    frame.data[1] = 0; // DriveMode=0
    frame.data[2] = 0; // All Relays Off
    send_blocking_command(frame);
    RCLCPP_INFO(logger_, "Sent Safety Deactivation Command.");
  }
  close_can_interface();
  return CallbackReturn::SUCCESS;
}

// ---------------- Read / Write Loop ----------------

return_type RovoCanSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!can_connected_) return return_type::OK;

  // Process all buffered messages
  while (true) {
    struct can_frame frame;
    const ssize_t nbytes = ::recv(can_socket_, &frame, sizeof(frame), MSG_DONTWAIT);

    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      RCLCPP_ERROR_THROTTLE(logger_, *get_clock(), 5000, "CAN Read Error: %s", std::strerror(errno));
      break;
    }
    
    if (static_cast<size_t>(nbytes) == sizeof(struct can_frame)) {
      process_can_frame(frame);
    } else {
      break;
    }
  }

  // Simple Dead Reckoning for Position
  const double dt = period.seconds();
  for (size_t i = 0; i < 2; ++i) {
    hw_positions_[i] += hw_velocities_[i] * dt;
  }

  return return_type::OK;
}

return_type RovoCanSystem::write(const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!can_connected_) return return_type::OK;
  
  last_write_time_ = time;

  uint8_t req_unlock = 0;
  uint8_t req_mode   = 0;

  // --- State Machine for Robust Activation ---
  switch (control_phase_) {
    case ExternalControlPhase::WAITING_FOR_READY:
      req_unlock = 0;
      req_mode   = 0;
      if (main_state_ == 2) {
        RCLCPP_INFO(logger_, "VCU Ready (MainState 2). Starting Handshake.");
        control_phase_ = ExternalControlPhase::REQUEST_STARTUP;
      }
      break;

    case ExternalControlPhase::REQUEST_STARTUP:
      req_unlock = 1;
      req_mode   = 0;
      if (unlock_state_ == 1) {
        control_phase_ = ExternalControlPhase::REQUEST_ACTIVE;
      }
      break;

    case ExternalControlPhase::REQUEST_ACTIVE:
      req_unlock = 2;
      req_mode   = 0;
      if (!password_sent_) {
        send_password_frame();
        password_sent_ = true;
      }
      if (unlock_state_ == 2) {
        RCLCPP_INFO(logger_, "VCU Active. Waiting for Drive Mode confirmation.");
        control_phase_ = ExternalControlPhase::WAITING_FOR_DRIVE_MODE;
      }
      break;

    case ExternalControlPhase::WAITING_FOR_DRIVE_MODE:
      req_unlock = 2;
      req_mode   = 2; // Request RPM Mode
      if (drive_mode_ == 2) {
        RCLCPP_INFO(logger_, "Drive Mode Confirmed. System Operational.");
        control_phase_ = ExternalControlPhase::ACTIVE;
      }
      break;

    case ExternalControlPhase::ACTIVE:
      req_unlock = 2;
      req_mode   = 2;
      break;
  }

  send_ec_tpdo1(req_unlock, req_mode);
  send_speed_frame();

  return return_type::OK;
}

// ---------------- Internal Logic ----------------

void RovoCanSystem::send_ec_tpdo1(uint8_t req_unlock, uint8_t req_mode)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id  = CAN_ID_EC_TPDO1;
  frame.can_dlc = 3;
  frame.data[0] = req_unlock;
  frame.data[1] = req_mode;
  
  // External HV is controlled by parameter. If true, we request it ON.
  // If false, we request it OFF (0), avoiding Error 01013.
  uint8_t hv_bit = enable_external_hv_ ? 1 : 0;
  
  frame.data[2] = (hv_bit << 2); 
  
  ::send(can_socket_, &frame, sizeof(frame), 0);
}

void RovoCanSystem::send_speed_frame()
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id  = CAN_ID_EC_TPDO2;
  frame.can_dlc = 8;
  
  uint8_t dir_left = 1, dir_right = 1;
  int16_t rpm_left = 0, rpm_right = 0;
  uint8_t stage = 0;

  if (control_phase_ == ExternalControlPhase::ACTIVE) {
    auto compute = [&](double cmd, uint8_t & dir, int16_t & rpm) {
        // Note: Inverted logic (-1.0) to match physical direction
        double m_rpm = (cmd ) * 60.0 / (2.0 * M_PI) * gear_ratio_;
        if (std::fabs(m_rpm) > 1e-3) {
            dir = (m_rpm >= 0) ? 1 : 2;
            rpm = static_cast<int16_t>(std::round(std::min(std::fabs(m_rpm), max_motor_rpm_)));
        }
    };
    compute(hw_commands_[0], dir_left, rpm_left);
    compute(hw_commands_[1], dir_right, rpm_right);

    // Smart Staging: Only engage gear (Stage 1) if motion is requested.
    // Otherwise, Idle (Stage 0) to allow brakes to engage.
    if (std::fabs(rpm_left) > 0 || std::fabs(rpm_right) > 0) {
        stage = (driving_stage_ > 0) ? driving_stage_ : 1;
    }
  }

  frame.data[0] = dir_left;
  frame.data[1] = dir_right;
  frame.data[2] = stage;
  
  // Little Endian Packing
  frame.data[3] = rpm_left & 0xFF;
  frame.data[4] = (rpm_left >> 8) & 0xFF;
  frame.data[5] = rpm_right & 0xFF;
  frame.data[6] = (rpm_right >> 8) & 0xFF;
  
  frame.data[7] = tx_alive_counter_++;

  ::send(can_socket_, &frame, sizeof(frame), 0);
}

void RovoCanSystem::process_can_frame(const struct can_frame & frame)
{
  const auto id = frame.can_id & CAN_SFF_MASK;

  switch (id) {
    case CAN_ID_EC_RPDO1:
      if (frame.can_dlc >= 8) {
        main_state_   = frame.data[0];
        unlock_state_ = frame.data[2];
        drive_mode_   = frame.data[6];
      }
      break;

    case CAN_ID_EC_RPDO2:
      if (frame.can_dlc >= 8) {
        // Unpack Speed (RPM)
        int16_t spd_l = static_cast<int16_t>(frame.data[0] | (frame.data[1] << 8));
        int16_t spd_r = static_cast<int16_t>(frame.data[2] | (frame.data[3] << 8));
        
        // Unpack Battery
        int16_t volt_raw = static_cast<int16_t>(frame.data[4] | (frame.data[5] << 8));
        int16_t curr_raw = static_cast<int16_t>(frame.data[6] | (frame.data[7] << 8));

        // Convert RPM -> Rad/s (and invert back to match command)
        const double factor = (2.0 * M_PI) / (60.0 * gear_ratio_) ;
        hw_velocities_[0] = static_cast<double>(spd_l) * factor;
        hw_velocities_[1] = static_cast<double>(spd_r) * factor;

        battery_voltage_ = static_cast<double>(volt_raw) * 0.1;
        battery_current_ = static_cast<double>(curr_raw) * 0.1;
      }
      break;

    case CAN_ID_ERR_EXT:
      if (frame.can_dlc >= 8 && (frame.data[0] & 0x01)) {
          static auto last_log_time = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
              RCLCPP_ERROR(logger_, "VCU Error (0x40A). FailCode: 0x%02X", frame.data[2]);
              last_log_time = now;
          }
      }
      break;
  }
}

void RovoCanSystem::send_password_frame()
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id  = CAN_ID_EC_PW_TX;
  frame.can_dlc = 6;
  for (size_t i = 0; i < 6; ++i) frame.data[i] = password_[i];
  ::send(can_socket_, &frame, sizeof(frame), 0);
}

bool RovoCanSystem::open_can_interface()
{
  if (can_connected_) return true;
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(logger_, "Socket create failed: %s", std::strerror(errno));
    return false;
  }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(logger_, "Interface %s not found", can_interface_name_.c_str());
    ::close(can_socket_);
    return false;
  }
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger_, "Bind failed");
    ::close(can_socket_);
    return false;
  }
  can_connected_ = true;
  return true;
}

void RovoCanSystem::close_can_interface()
{
  if (can_socket_ >= 0) ::close(can_socket_);
  can_connected_ = false;
}

void RovoCanSystem::send_blocking_command(const struct can_frame & frame)
{
  if (!can_connected_) return;
  ::send(can_socket_, &frame, sizeof(frame), 0);
}

}  // namespace rovo_can_driver

PLUGINLIB_EXPORT_CLASS(rovo_can_driver::RovoCanSystem, hardware_interface::SystemInterface)