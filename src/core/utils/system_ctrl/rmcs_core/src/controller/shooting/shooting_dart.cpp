#include <cmath>
#include <limits>
#include <rmcs_msgs/dart_shooting_mode.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include <rmcs_msgs/operate_mode.hpp>

namespace rmcs_core::controller::shooting {
class shooting_dartController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    shooting_dartController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter("joystick_left_bias_y", joystick_left_bias_y_);
        get_parameter("joystick_sensitivity", joystick_sensitivity_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        // register_input("/trigger/servo", trigger_servo_, false);
        // register_input("/trigger/servo_error", trigger_servo_error, false);
        // 0为左 1为右
        register_input("/trigger/left_motor/angle", trigger_motor_angle_[0]);
        register_output("/trigger/left_motor/control_angle", trigger_motor_control_angle[0]);

        register_input("/trigger/right_motor/angle", trigger_motor_angle_[1]);
        register_output("/trigger/right_motor/control_angle", trigger_motor_control_angle[1]);
        register_output("/trigger/mode", mode_, rmcs_msgs::dart_shooting_mode::SHOOT);
    }
    // void before_updating() override {
    //     if (!trigger_servo_.ready()) {
    //         trigger_servo_.make_and_bind_directly(0.0);
    //         RCLCPP_WARN(get_logger(), "Failed to fetch\"/trigger/servo\".Set to 0.0.");
    //     }
    //     if (!trigger_servo_error.ready()) {
    //         trigger_servo_error.make_and_bind_directly(0.0);
    //         RCLCPP_WARN(get_logger(), "Failed to fetch \"/trigger/servo_error\". Set to 0.0. ");
    //     }
    // }

    void update() override {
        using namespace rmcs_msgs;
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        if(!target_angle_initialized_){
            trigger_motor_target_angle[0] = (*trigger_motor_angle_[0]);
            trigger_motor_target_angle[1] = (*trigger_motor_angle_[1]);

            *trigger_motor_control_angle[0] = (trigger_motor_target_angle[0]);
            *trigger_motor_control_angle[1] = (trigger_motor_target_angle[1]);

            target_angle_initialized_ = true;
        }
        const bool remote_unavailable =
            switch_left == Switch::UNKNOWN
            || switch_right == Switch::UNKNOWN;

        const bool controls_disabled =
            switch_left == Switch::DOWN
            && switch_right == Switch::DOWN;
            
        
            if ((remote_unavailable)|| (controls_disabled)) {
                reset_all_controls();
                last_switch_right_ = switch_right;
                last_switch_left_ = switch_left;
                return;
                
            } 
            auto mode = *mode_;
            if (switch_left != Switch::DOWN) {
                if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == rmcs_msgs::dart_shooting_mode::RELOAD) {
                        mode = rmcs_msgs::dart_shooting_mode::STEP_DOWN;
                    } else {
                        mode = rmcs_msgs::dart_shooting_mode::RELOAD;
                    }
                }
                if(switch_left == Switch::MIDDLE && switch_right == Switch::UP){
                    mode = rmcs_msgs::dart_shooting_mode::SHOOT;
                }
                *mode_ = mode;
            }
            last_switch_right_ = switch_right;
            last_switch_left_ = switch_left;

      

       const double control_angle = ((*joystick_left_).y() - joystick_left_bias_y_ )*joystick_sensitivity_;
    
        trigger_motor_target_angle[0] += control_angle;
        trigger_motor_target_angle[1] -= control_angle;
        *trigger_motor_control_angle[0] = trigger_motor_target_angle[0];
        *trigger_motor_control_angle[1] = trigger_motor_target_angle[1];
    }

    void reset_all_controls() { *mode_ = rmcs_msgs::dart_shooting_mode::SHOOT; }


private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> trigger_motor_angle_[2];
    // InputInterface<double> trigger_servo_;
    // InputInterface<double> trigger_servo_error;
    OutputInterface<double> trigger_motor_control_angle[2];


    double trigger_motor_target_angle[2] = {0.0, 0.0};
    bool target_angle_initialized_ = false;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    double joystick_sensitivity_ = 0.004;

    OutputInterface<rmcs_msgs::dart_shooting_mode> mode_;
    double joystick_left_bias_y_ = 0.0;

};

} // namespace rmcs_core::controller::shooting



#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::shooting_dartController, rmcs_executor::Component)