#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
		client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
		while (!client_->wait_for_service(std::chrono::seconds(1)))
		{
			RCLCPP_WARN(this->get_logger(), "Waiting for the LED panel...");
		}
    }

private:
	rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
