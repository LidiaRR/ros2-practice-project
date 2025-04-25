#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#define PI 3.1415926535897

class TurtleBot : public rclcpp::Node
{
public:
	TurtleBot() : Node("turtlebot_controller")
	{
		vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
		auto subscriber_callback = [this](const turtlesim::msg::Pose::SharedPtr msg) {
			this->update_pose(msg);
		};
		pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
				"/turtle1/pose", 10, subscriber_callback);
		gotogoal();
	}
private:
	turtlesim::msg::Pose pose_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;

	void update_pose(const turtlesim::msg::Pose::SharedPtr msg)
	{
		pose_ = *msg;
	}

	double euclidean_distance(const turtlesim::msg::Pose &goal_pose)
	{
		return sqrt(pow(pose_.x - goal_pose.x, 2) + pow(pose_.y - goal_pose.y, 2));
	}

	double linear_vel(const turtlesim::msg::Pose &goal_pose, double constant = 1.5)
	{
		return constant * euclidean_distance(goal_pose);
	}

	double steering_angle(const turtlesim::msg::Pose &goal_pose)
	{
		return atan2(goal_pose.y - pose_.y, goal_pose.x - pose_.x);
	}

	double normalize_angle(double angle)
	{
		while (angle > PI) angle -= 2 * PI;
		while (angle < -PI) angle += 2 * PI;
		return angle;
	}

	double angular_vel(const turtlesim::msg::Pose &goal_pose, double constant = 2)
	{
		return constant * normalize_angle(steering_angle(goal_pose) - pose_.theta);
	}

	void gotogoal()
	{
		turtlesim::msg::Pose goal_pose;

		std::cout << "Set your x goal: ";
		std::cin >> goal_pose.x;

		std::cout << "Set your y goal: ";
		std::cin >> goal_pose.y;

		double tolerance;

		std::cout << "Set your tolerance: ";
		std::cin >> tolerance;

		geometry_msgs::msg::Twist vel_msg;
		
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;

		rclcpp::Rate loop_rate(10);

		while (rclcpp::ok() && euclidean_distance(goal_pose) >= tolerance)
		{	
			vel_msg.linear.x = linear_vel(goal_pose);
			vel_msg.angular.z = angular_vel(goal_pose);

			vel_publisher_->publish(vel_msg);

			rclcpp::spin_some(this->get_node_base_interface());

			loop_rate.sleep();
		}

		vel_msg.linear.x = 0;
		vel_msg.angular.z = 0;
		vel_publisher_->publish(vel_msg);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurtleBot>());
	rclcpp::shutdown();
	return 0;
}