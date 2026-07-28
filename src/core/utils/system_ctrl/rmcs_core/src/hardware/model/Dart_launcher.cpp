#include <cstdint>
#include <fast_tf/rcl.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
// #include "hardware/device/buzzer.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/servo.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class Dart_launcher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Dart_launcher()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<Dart_launcherCommend>(
                  get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_output("/tf", tf_);

        trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/trigger/calibrate", rclcpp::QoS{1}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_calibrate_subscription_callback(std::move(msg));
            });

        c_board_ = std::make_unique<CBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
    }

    ~Dart_launcher() override = default;

    void update() override {
        if (c_board_)
            c_board_->update();
    }

    void command_update() {
        if (c_board_)
            c_board_->command_update();
    }
    // 这是云台的控制程序包括日志输出和状态更新
private:
    void trigger_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (!c_board_) {
            RCLCPP_WARN(get_logger(), "Trigger calibrate requested but top board is  unavailable.");
            return;
        }
        RCLCPP_INFO(
            // 0为左 1为右
            get_logger(), "[trigger calibrate] New left offset: %d",
            c_board_->trigger_motor_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[trigger calibrate] New right offset: %d",
            c_board_->trigger_motor_[1].calibrate_zero_point());
    }
    class Dart_launcherCommend : public rmcs_executor::Component {
    public:
        explicit Dart_launcherCommend(Dart_launcher& dart_launcher)
            : dart_launcher(dart_launcher) {}

        void update() override { dart_launcher.command_update(); }

        Dart_launcher& dart_launcher;
    };
    std::shared_ptr<Dart_launcherCommend> command_component_;
    OutputInterface<rmcs_description::Tf> tf_;
    class CBoard final : private librmcs::client::CBoard {
    public:
        friend class Dart_launcher;
        explicit CBoard(
            Dart_launcher& dart_launcher, Dart_launcherCommend& dart_launchercommand,
            int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , tf_(dart_launcher.tf_)
            , dr16_(dart_launcher)
            // , buzzer_(dart_launchercommand)
            , supercap_(dart_launcher, 28.5)
            , trigger_motor_(
                  {dart_launcher, dart_launchercommand, "/trigger/left_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_encoder_zero_point(
                           static_cast<int>(
                               dart_launcher.get_parameter("left_motor_zero_point").as_int()))
                       .set_reduction_ratio(3591.0 / 187.0)
                       .enable_multi_turn_angle()},
                  {dart_launcher, dart_launchercommand, "/trigger/right_motor",
                   device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                       .set_encoder_zero_point(
                           static_cast<int>(
                               dart_launcher.get_parameter("right_motor_zero_point").as_int()))
                       .set_reduction_ratio(3591.0 / 187.0)
                       .enable_multi_turn_angle()

                  })
            // 替换 CBoard 构造函数初始化列表中的旧代码
            , trigger_servo_(
                  dart_launcher, dart_launchercommand, "/trigger/servo",
                  device::Servo::Servo_enable{device::Servo::Type::DG995})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {
            dart_launcher.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_multi(
                    [&buffer](std::byte byte) { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                transmit_buffer_.add_uart1_transmission(buffer, size);
                return size;
            };
        }

        ~CBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            *debug_imu_gx_bais_ = imu_.cali_gx_ref();
            *debug_imu_gy_bais_ = imu_.cali_gy_ref();
            *debug_imu_gz_bais_ = imu_.cali_gz_ref();

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());
            tf_->set_transform<rmcs_description::BaseLink, rmcs_description::RawImu>(
                gimbal_imu_pose);

            for (auto& motor : trigger_motor_)
                motor.update_status();
            supercap_.update_status();
            dr16_.update_status();
            // buzzer_.update_status();

            fast_tf::rcl::broadcast_all(*tf_);
        }

        void command_update() {
            uint16_t batch_commands[4];
            batch_commands[0] = trigger_motor_[0].generate_command();
            batch_commands[1] = trigger_motor_[1].generate_command();
            batch_commands[2] = 0;
            batch_commands[3] = 0;
            transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(batch_commands));

            // transmit_buffer_.add_buzzer_transmission(buzzer_.generate_command());

            transmit_buffer_.trigger_transmission();
        }

    private:
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if (can_id == 0x201) {
                trigger_motor_[0].store_status(can_data);

            } else if (can_id == 0x202) {
                trigger_motor_[1].store_status(can_data);
            } else if (can_id == 0x203) {
                uint8_t rx_data_0 = static_cast<uint8_t>(can_data & 0X200);
                trigger_servo_.handle_switch_cmd(rx_data_0, [](uint16_t /*pwm*/) {});
            } else if (can_id == 0x20c) {
                supercap_.store_status(can_data);
            }
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
        }
        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }
        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x - imu_bias_x, y - imu_bias_y, z - imu_bias_z);
        }
        void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            referee_ring_buffer_receive_.emplace_back_multi(
                [&uart_data](std::byte* storage) { *storage = *uart_data++; }, uart_data_length);
        }
        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;
        // device::Buzzer buzzer_;
        device::Supercap supercap_;
        int16_t imu_bias_x, imu_bias_y, imu_bias_z = 0.0;
        librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};

        OutputInterface<double> debug_imu_gx_bais_;
        OutputInterface<double> debug_imu_gy_bais_;
        OutputInterface<double> debug_imu_gz_bais_;

        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        device::DjiMotor trigger_motor_[2];
        device::Servo trigger_servo_;

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;

    std::unique_ptr<CBoard> c_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dart_launcher, rmcs_executor::Component)