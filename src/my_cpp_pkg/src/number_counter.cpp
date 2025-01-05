#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounter : public rclcpp::Node
{
public:
	NumberCounter() : Node("number_counter"), counter_(0)
	{
		subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
			"number", 10,
			std::bind(&NumberCounter::count, this, std::placeholders::_1));

		publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
		RCLCPP_INFO(this->get_logger(), "number_counter has been started.");

		server_ = this->create_service<example_interfaces::srv::SetBool>(
			"reset_counter",
			std::bind(&NumberCounter::callbackResetCount, this, std::placeholders::_1, std::placeholders::_2)
		);
	}
private:
	void count(const example_interfaces::msg::Int64::SharedPtr msg)
	{
		auto pub_msg = example_interfaces::msg::Int64();
		RCLCPP_INFO(this->get_logger(), "Received: %ld, Counter: %ld", msg->data, counter_);
		pub_msg.data = ++counter_;
		publisher_->publish(pub_msg);
	}

	void callbackResetCount(
		const example_interfaces::srv::SetBool::Request::SharedPtr request,
		const example_interfaces::srv::SetBool::Response::SharedPtr response)
	{
		if (request->data)
		{
			counter_ = 0;
			RCLCPP_INFO(this->get_logger(), "Counter Resetted");
			response->success = true;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Reset Aborted");
			response->success = false;
		}
			
	}
	rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
	rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
	int64_t counter_;
	rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
