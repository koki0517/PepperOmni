#include <chrono>
#include <memory>
#include <string>
#include <optional>
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>

#include "pepper_omni/logicool.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace PEPPER_OMNI {

class JoyPubNode : public rclcpp::Node
{
public:
	JoyPubNode()
	: Node("joy_pub")
	{
		pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		pepper_head_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pepper_head", 10);
		sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"joy",
			rclcpp::QoS(rclcpp::KeepLast(10)),
			std::bind(&JoyPubNode::joy_callback, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(), "joy_pub node started");
	}

private:
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		geometry_msgs::msg::Twist tw;
		float x = -msg->axes[0];
		float y =  msg->axes[1];
		tw.linear.x = x * 0.5; // max 0.5 m/s
		tw.linear.y = y * 0.5; // max 0.5 m/s
		// tw.angular.z = 
		pub_->publish(tw);

		// Publish a pepper_head value derived from joystick axes
		std_msgs::msg::Float64 ph;
		ph.data = -10.0 * msg->axes[static_cast<int>(Axes::LT)]; // 左-1,右+1
		pepper_head_pub_->publish(ph);
		RCLCPP_DEBUG_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "Published empty cmd_vel from joy message");
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pepper_head_pub_;
};

}  // namespace PEPPER_OMNI

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PEPPER_OMNI::JoyPubNode>());
	rclcpp::shutdown();
	return 0;
}