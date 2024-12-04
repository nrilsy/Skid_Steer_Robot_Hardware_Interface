#include "firmware/raspberryPi_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace ros2_control_hardware
{
DCMotorHardwareInterface::DCMotorHardwareInterface()
{
}
DCMotorHardwareInterface::~DCMotorHardwareInterface()  
{

}
hardware_interface::CallbackReturn DCMotorHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    if (info_.joints.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("DCMotorHardwareInterface"), "Expected exactly 2 joints (left and right)");
        return hardware_interface::CallbackReturn::ERROR;
    }

    wheels_.resize(info_.joints.size());
    // Adding left and write wheels by giving pins
    // Assuming it is a two joint system
    wheels_.at(0) = Wheel(5,6,13,wheel_radius_, "left");
    wheels_.at(1) = Wheel(17,27,12,wheel_radius_, "right");

    velocity_states_.resize(info_.joints.size());
    velocity_commands_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size()); // Initialize to zero


    if (info_.hardware_parameters.find("wheel_separation") == info_.hardware_parameters.end() ||
        info_.hardware_parameters.find("wheel_radius") == info_.hardware_parameters.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("DCMotorHardwareInterface"), "Missing hardware parameters: wheel_separation or wheel_radius.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
    
    for (const auto& joint : info_.joints) {
        RCLCPP_INFO(rclcpp::get_logger(""), "Joint found: %s", joint.name.c_str());
    }



    last_run_ = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {

    velocity_commands_ = {0.0, 0.0};
    velocity_states_ = {0.0, 0.0};
    position_states_ = {0.0, 0.0};

    try
    {    
        auto node = rclcpp::Node::make_shared("dc_motor_hardware_interface");
        odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DCMotorHardwareInterface::odomCallback, this, std::placeholders::_1));
        pi_ = pigpio_start(NULL, NULL);
        RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "Odom sub created, pigpio id is: %d", pi_);
    }
    catch(...)
    {
        RCLCPP_WARN(rclcpp::get_logger("hardware_interface"), "Could not initialize pigpio");
        return CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < wheels_.size(); i++)
    {
        wheels_.at(i).Write(pi_, 0);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {

    if (pi_ == 0)
    {
        try
        {
            for (size_t i = 0; i < wheels_.size(); i++)
            {
                wheels_.at(i).Write(pi_, 0);
            }
            pigpio_stop(pi_);
        }
        catch (...)
        {
            RCLCPP_WARN(rclcpp::get_logger("hardware_interface"), "Could not stop pigpio");
            return CallbackReturn::FAILURE;
        }
    }
    // CAN MAKE MOTORS STOP

    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
    return CallbackReturn::SUCCESS;

}

hardware_interface::return_type DCMotorHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    // Compute wheel velocities using odometry
    
    // odom_sub is continuously updates linear_vel_x and angular_vel_z values
    // so read function should take that information and use them to calculate
    // the motor velocities.
    // In this way without any encoder, velocity states can be read.
    // the formulation is :
    // For the left wheel                           For the right wheel
    // Vleft = Vx - Wz * (L/2)                      Vright = Vx + Wz * (L/2)
    // Vx is linear velocity, Wz iz angular velocity and L is wheel separation.
    // Update wheel positions using velocity and elapsed time
    for (size_t i = 0; i < velocity_states_.size(); i++)
    {
        // Integrate velocity to get position
        double elapsed_time = (rclcpp::Clock().now() - last_run_).seconds();
        position_states_.at(i) += velocity_states_.at(i) * elapsed_time;
    }

    // Example log to debug position values

    if (!(linear_velocity_x_ == 0 && angular_velocity_z_ == 0))
    {
        for (size_t i = 0; i < velocity_states_.size(); i++)
        {
            if (wheels_.at(i).GetSide() == "right")
            {
                velocity_states_.at(i) =  linear_velocity_x_ + angular_velocity_z_ * wheel_separation_ / 2.0;
            }
            else if ((wheels_.at(i).GetSide() == "left"))
            {
                velocity_states_.at(i) =  linear_velocity_x_ - angular_velocity_z_ * wheel_separation_ / 2.0;
            }
        }
    }
    last_run_ = rclcpp::Clock().now();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DCMotorHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    
    for (size_t i = 0; i < velocity_commands_.size(); i++)
    {
        if (std::abs(velocity_commands_.at(i)) > 0.8)
        {
            RCLCPP_WARN(rclcpp::get_logger("hardware_interface"), "Motors cannot operate above 0.8 m/s");
            return hardware_interface::return_type::ERROR;
        }
        else
        {
            wheels_.at(i).Write(pi_, velocity_commands_.at(i));
        }
    }
    
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DCMotorHardwareInterface::export_state_interfaces() {
    
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
            &velocity_states_.at(i)
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
            &position_states_.at(i)
        ));
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DCMotorHardwareInterface::export_command_interfaces() {
    
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
            &velocity_commands_.at(i)
        ));
    }

    return command_interfaces;
}

void DCMotorHardwareInterface::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    linear_velocity_x_ = msg->twist.twist.linear.x;
    angular_velocity_z_ = msg->twist.twist.angular.z;
}
} // namespace ros2_control_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_control_hardware::DCMotorHardwareInterface, hardware_interface::SystemInterface)
