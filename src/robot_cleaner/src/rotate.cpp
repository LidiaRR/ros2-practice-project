#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define PI 3.1415926535897

class RobotRotator : public rclcpp::Node
{
public:
	RobotRotator() : Node("robot_rotator")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
		rotate();
	}
private:
	void rotate()
	{
		geometry_msgs::msg::Twist vel_msg;
		double speed, angle;
		bool clockwise;

		std::cout << "Input your speed (deg/sec): ";
		std::cin >> speed;

		std::cout << "Input your angle (deg): ";
		std::cin >> angle;

		std::cout << "Clockwise? (1 true, 0 false) ";
		std::cin >> clockwise;

		speed = speed * 2 * PI / 360;
		angle = angle * 2 * PI / 360;

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = clockwise ? -speed : speed;		

		double current_angle = 0;
		auto start_time = this->now();
		rclcpp::Rate loop_rate(10);

		while (rclcpp::ok() && current_angle < angle)
		{
			publisher_->publish(vel_msg);
			auto now = this->now();
			current_angle = (now - start_time).seconds() * speed;
			loop_rate.sleep();
		}

		vel_msg.angular.z = 0.0;
		publisher_->publish(vel_msg);
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotRotator>());
	rclcpp::shutdown();
	return 0;
}