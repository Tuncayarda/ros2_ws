#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
		batt_fill_ = true;
		threads_.push_back(std::thread(std::bind(&BatteryNode::battery_loop, this)));
		RCLCPP_INFO(this->get_logger(), "Battery has been started");
    }

private:
	void battery_loop()
	{
		client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
		while (!client_->wait_for_service(std::chrono::seconds(1)))
		{
			RCLCPP_WARN(this->get_logger(), "Waiting for the LED panel...");
		}
		std::this_thread::sleep_for(std::chrono::seconds(4));
		batt_fill_ = false;
		auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
		request->led_number = 3;
		request->state = true;
		auto future = client_->async_send_request(request);
		std::this_thread::sleep_for(std::chrono::seconds(6));
		batt_fill_ = true;
		request->led_number = 3;
		request->state = false;
		future = client_->async_send_request(request);
	}
	rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
	std::vector<std::thread> threads_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool batt_fill_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
