#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel"), led_states_{0, 0, 0}
    {
		pub_ = this->create_publisher<my_robot_interfaces::msg::LedStates>(
			"led_panel_state",
			10
		);
		server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
			"set_led",
			std::bind(&LedPanelNode::sed_led_panel, this, _1, _2)
		);
		RCLCPP_INFO(this->get_logger(), "Service server has been started.");
		
		timer_ = this->create_timer(
			std::chrono::seconds(1),
			std::bind(&LedPanelNode::publishLedStates, this)
		);
		RCLCPP_INFO(this->get_logger(), "LedPanel has been started");
    }

private:
	void sed_led_panel(
	const my_robot_interfaces::srv::SetLed::Request::SharedPtr request, 
	const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
	{
		if (request->led_number > (int64_t)led_states_.size() || request->led_number < 0)
		{
			RCLCPP_INFO(this->get_logger(), "Exceed limit");
			response->success = false;
			return ;
		}

		if (request->state)
			led_states_[request->led_number - 1] = 1;
		else
			led_states_[request->led_number - 1] = 0;
		response->success = true;
	}

	void publishLedStates()
	{
		auto msg = my_robot_interfaces::msg::LedStates();
		msg.led_states = std::vector<int64_t>(std::begin(led_states_), std::end(led_states_));
		pub_->publish(msg);
	}

	rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
	rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::array<int64_t, 3> led_states_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
