#include "cloud_robot/servo_hw.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>  
#include <iomanip> 

namespace cloud_robot{

static rclcpp::Logger hw_logger = rclcpp::get_logger("ServoHW");

hardware_interface::CallbackReturn ServoHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t n = info.joints.size();
  servos_.resize(n);
  current_pos_.assign(n, 0.0);
  current_pos_vis_.assign(n, 0.0);
  goal_pos_.assign(n, 0.0);
  current_vel_.assign(n, 0.0);
  last_us_.assign(n, 0);

  for (size_t i = 0; i < n; i++)
  {
    const auto & joint = info.joints[i];
    servos_[i].joint_name = joint.name;
    servos_[i].channel = std::stoi(joint.parameters.at("channel"));
    servos_[i].rad_min = std::stod(joint.parameters.at("rad_min"));
    servos_[i].rad_max = std::stod(joint.parameters.at("rad_max"));
    servos_[i].us_min = std::stoi(joint.parameters.at("us_min"));
    servos_[i].us_max = std::stoi(joint.parameters.at("us_max"));
    servos_[i].zero_offset_rad = std::stod(joint.parameters.at("zero_offset_rad"));
    servos_[i].invert = joint.parameters.at("invert") == "true";
    servos_[i].max_velocity = std::stod(joint.parameters.at("max_velocity"));
    servos_[i].max_acceleration = std::stod(joint.parameters.at("max_acceleration"));

   RCLCPP_INFO(hw_logger,
    "Loaded joint %s | channel=%d | invert=%d | vel=%.2f ",
    servos_[i].joint_name.c_str(),
    servos_[i].channel,
    servos_[i].invert,
    servos_[i].max_velocity
    );
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ServoHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < servos_.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        servos_[i].joint_name, "position", &current_pos_vis_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ServoHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < servos_.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        servos_[i].joint_name, "position", &goal_pos_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ServoHW::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!driver_connect())
  {
    RCLCPP_ERROR(hw_logger, "Failed to connect servo driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < servos_.size(); i++)
  {
    goal_pos_[i] = 0.0;
    servo_direct(i, goal_pos_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoHW::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < servos_.size(); i++)
  {
    goal_pos_[i] = 0.0;
    servo_direct(i, goal_pos_[i]);
  }

  driver_disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// DRIVER CONNECTION
bool ServoHW::driver_connect()
{
  port_ = "/dev/ttyACM0";

  for (int i = 0; i < 3; i++)
  {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    
    if (serial_fd_ >= 0)
    {
      RCLCPP_INFO(hw_logger, "Connected to servo driver");
      return true;
    }

    RCLCPP_WARN(hw_logger, "Port not ready, retrying... (%d)", i);
    sleep(1);
  }

  RCLCPP_ERROR(hw_logger, "Cannot open port %s", port_.c_str());
  return false;
}

bool ServoHW::driver_disconnect()
{
  if (serial_fd_ >= 0) close(serial_fd_);
  serial_fd_ = -1;
  return true;
}

hardware_interface::return_type ServoHW::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type ServoHW::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    double dt = period.seconds();
    if (dt <= 0.0) return hardware_interface::return_type::OK;

    // LIMITAR FRECUENCIA A ~75 Hz
    static double accum = 0.0;
    accum += dt;

    if (accum < 0.0133)
        return hardware_interface::return_type::OK;

    accum = 0.0;

    for (size_t i = 0; i < servos_.size(); ++i)
    {
        const auto & s = servos_[i];

        double error = goal_pos_[i] - current_pos_[i];
        double dist  = std::fabs(error);
        int sign     = (error >= 0) ? 1 : -1;

        double v_brake  = std::sqrt(2.0 * s.max_acceleration * dist);
        double v_target = std::min(s.max_velocity, v_brake);

        double dv_max = s.max_acceleration * dt;
        double v_cmd  = sign * v_target;

        if (v_cmd > current_vel_[i] + dv_max)
            current_vel_[i] += dv_max;
        else if (v_cmd < current_vel_[i] - dv_max)
            current_vel_[i] -= dv_max;
        else
            current_vel_[i] = v_cmd;

        double step = current_vel_[i] * dt;

        if (std::fabs(step) > dist)
            step = sign * dist;

        current_pos_[i] += step;

        // FILTRO
        uint16_t new_us = cmd_rad_to_us(servos_[i], current_pos_[i]);

        if (std::abs((int)new_us - (int)last_us_[i]) >= 5)
        {
            servo_direct(i, current_pos_[i]);
        }

        current_pos_vis_[i] = current_pos_[i];

        if (servos_[i].invert){
          current_pos_vis_[i] = -current_pos_vis_[i];
        }
    }

    return hardware_interface::return_type::OK;
}

bool ServoHW::driver_send(const std::vector<uint8_t>& data)
{
  if (serial_fd_ < 0) return false;

  ssize_t written = ::write(serial_fd_, data.data(), data.size());

  if (written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
        RCLCPP_WARN(hw_logger, "Serial busy, skipping frame");
        return false;
    }

    RCLCPP_ERROR(hw_logger, "Serial write error: errno=%d (%s)",
                 errno, strerror(errno));
    return false;
  }

  if (written != static_cast<ssize_t>(data.size())){ 
    RCLCPP_WARN(hw_logger, "Partial write");
    return false;
  }

  return true;
}

bool ServoHW::set_servo_us(uint8_t channel, uint16_t us)
{
  uint16_t target = us * 4;
  std::vector<uint8_t> cmd = {
    0x84,
    channel,
    static_cast<uint8_t>(target & 0x7F),
    static_cast<uint8_t>((target >> 7) & 0x7F)
  };
  return driver_send(cmd);
}

uint16_t ServoHW::cmd_rad_to_us(const Servo & s, double rad_cmd){
  if (s.invert) rad_cmd = -rad_cmd;
  rad_cmd = std::clamp(rad_cmd, s.rad_min, s.rad_max);
  double rad_real = rad_cmd + s.zero_offset_rad;
  double norm = rad_real / (s.rad_max - s.rad_min);
  uint16_t us = s.us_min + norm * (s.us_max - s.us_min);

  RCLCPP_DEBUG(hw_logger, "%s | rad=%.2f | us=%d",
               s.joint_name.c_str(),
               rad_cmd,
               us);
  return us;
}

void ServoHW::servo_direct(size_t i, double target_rad)
{
    uint16_t us = cmd_rad_to_us(servos_[i], target_rad);

    if (set_servo_us(servos_[i].channel, us)) {
        last_us_[i] = us;
    }
}
} // namespace cloud_robot

PLUGINLIB_EXPORT_CLASS(
  cloud_robot::ServoHW,
  hardware_interface::SystemInterface)
