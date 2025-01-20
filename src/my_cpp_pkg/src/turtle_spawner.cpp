#include <random>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp" 
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
	TurtleSpawnerNode() : Node("turtle_spawner"), rng_(std::random_device{}())
	{
		publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("/alive_turtles", 10);
		client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
		kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
		while (!client_->wait_for_service(std::chrono::seconds(1)))
			RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
		while (!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_INFO(this->get_logger(), "Waiting for the 'kill' service to become available...");
		timer_ = this->create_timer(
			std::chrono::seconds(5),
			std::bind(&TurtleSpawnerNode::spawn_turtle, this)
		);
		catch_server_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
			"catch_turtle",
			std::bind(&TurtleSpawnerNode::catch_turtle_callback, this, _1, _2)
		);
		publish_timer_ = this->create_timer(
			std::chrono::milliseconds(500),
			std::bind(&TurtleSpawnerNode::publish_alive_turtles, this)
		);
		RCLCPP_INFO(this->get_logger(), "Turtle Spawner has been started.");
	}

private:
	void catch_turtle_callback(
		const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request, 
		const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
		{
			auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
			kill_request->name = request->name;
			kill_client_->async_send_request(kill_request);
			turtles_.erase(request->name);
			response->success = true;
			publish_alive_turtles();
		}

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
            [this, request](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) {
                try
                {
                    RCLCPP_INFO(this->get_logger(), "Turtle spawned successfully: %s", response.get()->name.c_str());
					turtles_[request->name] = {request->x, request->y};
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error spawning turtle: %s", e.what());
                }
            });
	}
	
	void publish_alive_turtles()
	{
		auto turtle_array_msg = my_robot_interfaces::msg::TurtleArray();
		for (const auto &[name, position] : turtles_)
        {
            my_robot_interfaces::msg::Turtle turtle_msg;
            turtle_msg.name = name;
            turtle_msg.x = position.first;
            turtle_msg.y = position.second;
            turtle_array_msg.turtles.push_back(turtle_msg);
        }

        publisher_->publish(turtle_array_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu turtles.", turtle_array_msg.turtles.size());
	}

	std::mt19937 rng_;
	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr publish_timer_;
	rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
	rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr publisher_;
	rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_server_;
	std::map<std::string, std::pair<double, double>> turtles_;

	int turtle_counter_ = 100;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleSpawnerNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
