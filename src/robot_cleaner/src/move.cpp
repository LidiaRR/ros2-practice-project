#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotMover : public rclcpp::Node
{
public:
	RobotMover() : Node("robot_mover")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
		move();
	}
private:
	void move()
	{
		geometry_msgs::msg::Twist vel_msg;
		double speed, distance;
		bool is_forward;

		std::cout << "Input your speed: ";
		std::cin >> speed;

		std::cout << "Input your distance: ";
		std::cin >> distance;

		std::cout << "Forward? (1 true, 0 false) ";
		std::cin >> is_forward;

		vel_msg.linear.x = is_forward ? speed : -speed;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;		

		double current_dist = 0;
		auto start_time = this->now();
		rclcpp::Rate loop_rate(10);

		while (rclcpp::ok() && current_dist < distance)
		{
			publisher_->publish(vel_msg);
			auto now = this->now();
			current_dist = (now - start_time).seconds() * speed;
			loop_rate.sleep();
		}

		vel_msg.linear.x = 0.0;
		publisher_->publish(vel_msg);
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotMover>());
	rclcpp::shutdown();
	return 0;
}