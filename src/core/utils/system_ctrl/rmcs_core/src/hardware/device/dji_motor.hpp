#pragma once

#include "rmcs_utility/tick_timer.hpp"
#include "utility/low_pass_filter.hpp"

#include <cstdint>
#include <librmcs/device/dji_motor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class DjiMotor : public librmcs::device::DjiMotor {
public:
// DjiMotor类是一个继承自librmcs::device::DjiMotor的类，它在构造函数中注册了一些输出接口和输入接口，
// 这些接口用于在系统控制器中发布电机的状态信息和接收控制命令。
// 它还包含了一个alive_watchdog_成员变量，用于监测电机的在线状态，
// 以及一个velocity_lpf_成员变量，用于对电机的速度进行低通滤波处理。
    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::DjiMotor() {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle", raw_angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);
        status_component.register_output(
            name_prefix + "/velocity_filtered", velocity_filtered_, 0.0);
        status_component.register_output(name_prefix + "/alive", alive_, false);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);

        motor_name_ = name_prefix;
        alive_watchdog_.reset(50);
    }
// DjiMotor类还定义了一个configure函数，用于配置电机的参数，一个update_status函数，用于更新电机的状态信息，
// 一个store_status函数，用于存储从CAN消息中解析出的电机状态信息，一个control_torque函数，用于获取当前的控制扭矩值，
// 以及一个generate_command函数，用于生成发送给电机的CAN命令数据。
    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DjiMotor(status_component, command_component, name_prefix) {
        configure(config);
    }
// DjiMotor类的configure函数会调用基类的configure函数来配置电机的参数，然后将最大扭矩值发布到对应的输出接口上。
    void configure(const Config& config) {
        librmcs::device::DjiMotor::configure(config);

        *max_torque_ = max_torque();
    }
// DjiMotor类的update_status函数会定期被系统控制器调用，在这个函数中可以更新电机的状态信息，
// 例如读取电机的角度、速度、扭矩等数据，并发布到对应的输出接口上。
    void update_status() {
        librmcs::device::DjiMotor::update_status();

        if (alive_watchdog_.tick()) {
            *alive_ = false;
            RCLCPP_WARN(
                rclcpp::get_logger("HW_Diag"), "Dji Motor %s offline!", motor_name_.c_str());
        }
        *angle_ = angle();
        *raw_angle_ = last_raw_angle();
        *velocity_ = velocity();
        *torque_ = torque();
        *velocity_filtered_ = velocity_lpf_.update(velocity());
    }
// DjiMotor类的store_status函数会被CAN消息接收回调函数调用，在这个函数中可以解析从CAN消息中提取出的电机状态信息，
    void store_status(uint64_t can_data) {
        librmcs::device::DjiMotor::store_status(can_data);

        *alive_ = true;
        alive_watchdog_.reset(50);
    }
// DjiMotor类的control_torque函数会被命令生成函数调用，在这个函数中可以获取当前的控制扭矩值，
// 这个值是从系统控制器的输入接口上接收的，可以根据实际情况来设置默认值或者进行一些处理。
    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }
// DjiMotor类的generate_command函数会被系统控制器调用，在这个函数中可以生成发送给电机的CAN命令数据，
// 这个函数会调用基类的generate_command函数来生成命令数据，传递当前的控制扭矩值作为参数。
    uint16_t generate_command() {
        return librmcs::device::DjiMotor::generate_command(control_torque());
    }
// DjiMotor类的can_receive_callback函数会被CAN消息接收回调函数调用，在这个函数中可以根据接收到的CAN消息ID来解析不同类型的电机状态信息，
// 例如根据CAN消息ID来区分是电机的角度信息、速度信息还是扭矩信息，并将解析出的数据存储在对应的成员变量中。
private:
    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> raw_angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> velocity_filtered_;
    rmcs_executor::Component::OutputInterface<double> torque_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;
    rmcs_executor::Component::OutputInterface<bool> alive_;

    rmcs_executor::Component::InputInterface<double> control_torque_;

    std::string motor_name_;
    rmcs_utility::TickTimer alive_watchdog_;
    rmcs_core::utility::LowPassFilter<> velocity_lpf_{4, 1000};
};

} // namespace rmcs_core::hardware::device