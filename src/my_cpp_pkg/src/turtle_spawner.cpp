#include <random>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
	TurtleSpawnerNode() : Node("turtle_spawner"), rng_(std::random_device{}())
	{
		client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
		while (!client_->wait_for_service(std::chrono::seconds(1)))
			RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
		timer_ = this->create_timer(
			std::chrono::seconds(4),
			std::bind(&TurtleSpawnerNode::spawn_turtle, this)
		);
		RCLCPP_INFO(this->get_logger(), "Turtle Spawner has been started.");
	}

private:
	void spawn_turtle()
	{
		std::uniform_real_distribution<double> dist_loc(0.0, 11.0);
		std::uniform_real_distribution<double> dist_angle(0.0, 90.0);
		auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
		request->x = dist_loc(rng_);
		request->y = dist_loc(rng_);
		request->theta = dist_angle(rng_);
		request->name = "rand_turtle" + std::to_string(turtle_counter_++);

        auto future = client_->async_send_request(request,
            [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) {
                try
                {
                    RCLCPP_INFO(this->get_logger(), "Turtle spawned successfully: %s", response.get()->name.c_str());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error spawning turtle: %s", e.what());
                }
            });
	}
	std::mt19937 rng_;
	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
	int turtle_counter_ = 1;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleSpawnerNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
