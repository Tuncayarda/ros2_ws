#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rcu_spi_comm/msg/motor_servo.hpp"

using std::placeholders::_1;

// This is the class for the ROS2 node that handles joystick input and sends motor/servo data
class JoyToMotorServoNode : public rclcpp::Node
{
public:
    JoyToMotorServoNode() : Node("joy_to_motorservo_node")
    {
        // Creating a subscription to the joystick topic ("joy") with a callback function
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyToMotorServoNode::joy_callback, this, _1));

        // Creating a publisher for motor/servo data to send it to the corresponding topic
        motor_servo_pub_ = this->create_publisher<rcu_spi_comm::msg::MotorServo>("motor_servo_data", 10);
    }

private:
    int mode_ = 0;  // Variable to store the current mode (0 or 1)
    bool prev_button_state_ = false;  // To track the previous state of the button

    // Callback function that processes joystick data
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if the button 3 (index 3) is pressed to switch modes
        bool current_button_state = msg->buttons.size() > 3 && msg->buttons[3] == 1;
        if (current_button_state && !prev_button_state_) {
            mode_ = (mode_ + 1) % 2;  // Toggle between mode 0 and 1
            RCLCPP_INFO(this->get_logger(), "Mode switched to: %d", mode_);
        }
        prev_button_state_ = current_button_state;  // Update previous button state

        // Mode 0: Handle the joystick input for driving and steering
        if (mode_ == 0) {
            auto motor_servo_msg = rcu_spi_comm::msg::MotorServo();

            // Extract the joystick axis value for steering (axis 3)
            float axis_value = 0.0f;
            if (!msg->axes.empty()) {
                axis_value = msg->axes[3];
            }

            // Convert joystick axis value to servo angle in the range of -45 to 45 degrees
            int8_t servo_angle = static_cast<int8_t>(axis_value * 45.0f);

            // Define the inner and outer servo angles based on the turn direction
            int8_t inner_angle = static_cast<int8_t>(servo_angle * 0.5f);
            int8_t outer_angle = servo_angle;

            // Assign servo values based on turning direction
            if (servo_angle > 0) {  // Turning right
                motor_servo_msg.servos[0] = -outer_angle;
                motor_servo_msg.servos[1] = 0;
                motor_servo_msg.servos[2] = outer_angle;
                motor_servo_msg.servos[3] = -inner_angle;
                motor_servo_msg.servos[4] = 0;
                motor_servo_msg.servos[5] = inner_angle;
            } else {  // Turning left or straight
                motor_servo_msg.servos[0] = -inner_angle;
                motor_servo_msg.servos[1] = 0;
                motor_servo_msg.servos[2] = inner_angle;
                motor_servo_msg.servos[3] = -outer_angle;
                motor_servo_msg.servos[4] = 0;
                motor_servo_msg.servos[5] = outer_angle;
            }

            // Handle throttle based on the joystick axis for forward/reverse movement
            float axis2 = msg->axes.size() > 2 ? msg->axes[2] : 1.0f;
            float axis5 = msg->axes.size() > 5 ? msg->axes[5] : 1.0f;

            // Normalize press amount to range between 0 (released) and 1 (fully pressed)
            float press2 = (1.0f - axis2) / 2.0f;  // Reverse throttle
            float press5 = (1.0f - axis5) / 2.0f;  // Forward throttle

            float diff = press5 - press2;  // Calculate the difference for motor power

            // Set motor power to control forward/reverse movement
            int16_t motor_power = static_cast<int16_t>(diff * 255.0f);

            // Set the motor values for all motors
            for (size_t i = 0; i < 6; i++) {
                motor_servo_msg.motors[i] = motor_power;
            }

            // Publish the motor and servo values to the motor_servo_data topic
            motor_servo_pub_->publish(motor_servo_msg);
        }

        // Mode 1: Handle different motor/servo behavior (could be a custom mode)
        if (mode_ == 1) {
            RCLCPP_INFO(this->get_logger(), "Mode 1 active");

            // Example: set predefined motor power values for mode 1
            auto motor_servo_msg = rcu_spi_comm::msg::MotorServo();

            motor_servo_msg.servos[0] = 55;
            motor_servo_msg.servos[1] = 0;
            motor_servo_msg.servos[2] = -55;
            motor_servo_msg.servos[3] = -55;
            motor_servo_msg.servos[4] = 0;
            motor_servo_msg.servos[5] = 55;

            // Handle throttle for forward/reverse in mode 1
            float axis2 = msg->axes.size() > 2 ? msg->axes[2] : 1.0f;
            float axis5 = msg->axes.size() > 5 ? msg->axes[5] : 1.0f;

            float press2 = (1.0f - axis2) / 2.0f;
            float press5 = (1.0f - axis5) / 2.0f;

            float diff = press5 - press2;
            int16_t motor_power = static_cast<int16_t>(diff * 255.0f);

            // Set motor power for each motor based on mode 1 logic
            for (size_t i = 0; i < 3; i++) {
                motor_servo_msg.motors[i] = -motor_power;
            }
            for (size_t i = 3; i < 6; i++) {
                motor_servo_msg.motors[i] = motor_power;
            }

            // Publish motor/servo data in mode 1
            motor_servo_pub_->publish(motor_servo_msg);
        }
    }

    // Subscriber for joystick messages
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Publisher for motor/servo data
    rclcpp::Publisher<rcu_spi_comm::msg::MotorServo>::SharedPtr motor_servo_pub_;
};

// Main function that initializes ROS2, creates the node, and spins it
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    rclcpp::spin(std::make_shared<JoyToMotorServoNode>());  // Spin the node to process callbacks
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;  // Exit the program
}