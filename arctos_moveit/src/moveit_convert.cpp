#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arctos_moveit/msg/arm_joint_state.hpp"
#include <cmath>

class MoveitConverter : public rclcpp::Node
{
public:
    MoveitConverter() : Node("moveit_converter"), count_(0), joint_status_(0)
    {
        // Initialize arrays
        for (int i = 0; i < 6; i++) {
            cur_angle_[i] = 0.0;
            joint_step_[i] = 0;
            prev_angle_[i] = 0.0;
            init_angle_[i] = 0.0;
            total_steps_[i] = 0.0;
        }
        
        // Initialize steps per revolution array
        steps_per_revolution_[0] = 32800;  // microsteps/revolution for joint1
        steps_per_revolution_[1] = 18000;  // microsteps/revolution for joint2
        steps_per_revolution_[2] = 72000;  // microsteps/revolution for joint3
        steps_per_revolution_[3] = 3280;   // microsteps/revolution for joint4
        steps_per_revolution_[4] = 14400;  // microsteps/revolution for joint5
        steps_per_revolution_[5] = 0;      // microsteps/revolution for joint6
        
        // Create subscription to fake controller joint states
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/move_group/fake_controller_joint_states", 10,
            std::bind(&MoveitConverter::cmd_callback, this, std::placeholders::_1));
        
        // Create publisher for joint steps
        publisher_ = this->create_publisher<arctos_moveit::msg::ArmJointState>("joint_steps", 10);
        
        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MoveitConverter::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Moveit Converter Node Started");
    }

private:
    void cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (count_ == 0) {
            for (int i = 0; i < 6 && i < static_cast<int>(msg->position.size()); i++) {
                prev_angle_[i] = msg->position[i];
                init_angle_[i] = msg->position[i];
            }
        }

        RCLCPP_INFO(this->get_logger(), "Received /move_group/fake_controller_joint_states");
        
        // Compute relative step count to move each joint
        for (int i = 0; i < 6 && i < static_cast<int>(msg->position.size()); i++) {
            double angle_diff = msg->position[i] - prev_angle_[i];
            joint_step_[i] = static_cast<int>(angle_diff * steps_per_revolution_[i] / (2 * M_PI));
        }

        if (count_ != 0) {
            for (int i = 0; i < 6 && i < static_cast<int>(msg->position.size()); i++) {
                prev_angle_[i] = msg->position[i];
            }
        }

        // Update total steps
        total_.position1 += joint_step_[0];
        total_.position2 += joint_step_[1];
        total_.position3 += joint_step_[2];
        total_.position4 += joint_step_[3];
        total_.position5 += joint_step_[4];
        total_.position6 += joint_step_[5];

        RCLCPP_INFO(this->get_logger(), "Done conversion to /joint_steps");
        joint_status_ = 1;
        count_ = 1;
    }
    
    void timer_callback()
    {
        if (joint_status_ == 1) {
            joint_status_ = 0;
            publisher_->publish(total_);
            RCLCPP_INFO(this->get_logger(), "Published to /joint_steps");
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<arctos_moveit::msg::ArmJointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    arctos_moveit::msg::ArmJointState arm_steps_;
    arctos_moveit::msg::ArmJointState total_;
    int steps_per_revolution_[6];
    int joint_status_;
    double cur_angle_[6];
    int joint_step_[6];
    double prev_angle_[6];
    double init_angle_[6];
    double total_steps_[6];
    int count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
