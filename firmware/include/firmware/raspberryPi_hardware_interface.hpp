#ifndef DC_MOTOR_HARDWARE_INTERFACE_HPP
#define DC_MOTOR_HARDWARE_INTERFACE_HPP

#include "firmware/wheel.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <string>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace ros2_control_hardware
{
class DCMotorHardwareInterface : public hardware_interface::SystemInterface {
public:
    DCMotorHardwareInterface();
    ~DCMotorHardwareInterface();

    // Overridden methods
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    // Export interfaces
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:

    // Robot parameters
    double wheel_separation_;
    double wheel_radius_;

    std::vector<Wheel> wheels_;

    // Command and state variables
    std::vector<double> velocity_commands_;
    std::vector<double> velocity_states_;
    std::vector<double> position_states_;


    // ROS2 Subscription
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);


    // Variables to store odometry-derived velocities
    double linear_velocity_x_;
    double angular_velocity_z_;
    

    int pi_;
    
    rclcpp::Time last_run_;
};
} // namespace ros2_control_hardware
#endif
