#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

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
	}
private:
	void count(const example_interfaces::msg::Int64::SharedPtr msg)
	{
		auto pub_msg = example_interfaces::msg::Int64();
		RCLCPP_INFO(this->get_logger(), "Received: %ld, Counter: %ld", msg->data, counter_);
		pub_msg.data = ++counter_;
		publisher_->publish(pub_msg);
	}

	rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
	rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
	int64_t counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}