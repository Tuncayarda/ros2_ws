#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

#include <cmath>

using std::placeholders::_1;

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel",
            10
        );
        client_ = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControllerNode::callback_turtle_pose, this, _1)
        );
        turtle_array_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "alive_turtles",
            10,
            std::bind(&TurtleControllerNode::callback_alive_turtles, this, _1)
        );
		timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TurtleControllerNode::control_loop, this)
        );
    }

private:
    void callback_alive_turtles(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        this->alive_turtles_ = msg;
        
    }

    void callback_turtle_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        this->pose_ = msg;
    }

    void control_loop()
    {
        int i = 0;
        if (!pose_ || !alive_turtles_)
            return;

        if (alive_turtles_ && !alive_turtles_->turtles.empty())
        {
            this->target_x_ = this->alive_turtles_->turtles[i].x;
            this->target_y_ = this->alive_turtles_->turtles[i].y;
        }
        double dist_x = this->target_x_ - this->pose_->x;
        double dist_y = this->target_y_ - this->pose_->y;
        double distance = sqrt(dist_x * dist_x + dist_y * dist_y);

        geometry_msgs::msg::Twist cmd_val;

        if (distance > 0.5)
        {
            cmd_val.linear.x = distance;

            double goal_theta = atan2(dist_y, dist_x);
            double diff = goal_theta - this->pose_->theta;

            if (diff > M_PI)
                diff -= 2 * M_PI;
            else if (diff < -M_PI)
                diff += 2 * M_PI;

            cmd_val.angular.z = 6 * diff;
        }
        else
        {
            auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
            request->name = this->alive_turtles_->turtles[0].name;
            client_->async_send_request(request);
            cmd_val.linear.x = 0.0;
            cmd_val.angular.z = 0.0;
            i++;
        }

        this->publisher_->publish(cmd_val);
    }

    turtlesim::msg::Pose::SharedPtr pose_;
    my_robot_interfaces::msg::TurtleArray::SharedPtr alive_turtles_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr turtle_array_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr client_;

    rclcpp::TimerBase::SharedPtr timer_; 

    double target_x_ = 0;
    double target_y_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
