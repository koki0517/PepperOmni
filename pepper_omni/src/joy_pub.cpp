#include <chrono>
#include <memory>
#include <string>
#include <optional>
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>

#include "pepper_omni/logicool.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "uec_msgs/msg/odometry.hpp"
#include "pepper_msgs/msg/pepper_pos.hpp"
#include "uec_utils/uec_utils.hpp"

/* 頭の胴体に対する初期位置 668 (0~1023)
 */

namespace PEPPER_OMNI {

class JoyPubNode : public rclcpp::Node
{
public:
	// MARK: constructor
	JoyPubNode(): Node("joy_pub")
	{
		pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		pepper_head_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pepper_head", 10);
		sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"joy",
			rclcpp::QoS(rclcpp::KeepLast(10)),
			std::bind(&JoyPubNode::joy_callback, this, std::placeholders::_1));

		// subscribe to /uodom (uec_msgs::msg::Odometry)
		uodom_sub_ = this->create_subscription<uec_msgs::msg::Odometry>(
			"/uodom",
			rclcpp::QoS(rclcpp::KeepLast(10)),
			std::bind(&JoyPubNode::uodom_callback, this, std::placeholders::_1));
		
		RCLCPP_INFO(this->get_logger(), "joy_pub node started");
		// subscribe to /pepper_pos (pepper_msgs::msg::PepperPos)
		pepper_pos_sub_ = this->create_subscription<pepper_msgs::msg::PepperPos>(
			"/pepper_pos",
			rclcpp::QoS(rclcpp::KeepLast(10)),
			std::bind(&JoyPubNode::pepper_pos_callback, this, std::placeholders::_1));
		
		head_pid_.init(1.0, 0.0, 0.0, M_PI_2, 0.5); // PIDパラメータは適宜調整
		base_pid_.init(1.20, 0.0, 0.5, M_PI_2, M_PI*4); // PIDパラメータは適宜調整
	}

private:
	// MARK: joy cb
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		geometry_msgs::msg::Twist tw;
		static bool last_LBRB_status = false;
		float x = -msg->axes[0];
		float y =  msg->axes[1];
		tw.linear.x = x * 0.5; // max 0.5 m/s
		tw.linear.y = y * 0.5; // max 0.5 m/s
		// 胴体の回転
		if (msg->buttons[static_cast<int>(Buttons::LB)] == 1){
			tw.angular.z = -M_PI_2; // 左回転
		} else if (msg->buttons[static_cast<int>(Buttons::RB)] == 1){
			tw.angular.z = M_PI_2; // 右回転
		}
		if (last_LBRB_status && (msg->buttons[static_cast<int>(Buttons::LB)] == 1 || msg->buttons[static_cast<int>(Buttons::RB)] == 1)){
			// 回り終わり
			goal_base_pos_map_rad_ = yaw_;
		} else {
			// body PID
			float e = normalizeAngle(goal_base_pos_map_rad_ - yaw_);
			tw.angular.z = -base_pid_.update(e);
		}
		pub_->publish(tw);
		last_LBRB_status = (msg->buttons[static_cast<int>(Buttons::LB)] == 1 || msg->buttons[static_cast<int>(Buttons::RB)] == 1);

		// 頭の回転
		std_msgs::msg::Float64 ph;
		static bool last_RX_status = false;
		ph.data = -10.0 * msg->axes[static_cast<int>(Axes::LT)]; // 左-1,右+1
		
		if (msg->axes[static_cast<int>(Axes::LT)] < 0.1 && last_RX_status){
			// 回り終わり
			goal_head_pos_map_rad_ = ph.data;
		}
		else if (msg->axes[static_cast<int>(Axes::LT)] < 0.1){
			//head PID
			float head_pos_map_rad = normalizeAngle(head_pos_base_rad_ + yaw_);
			float e = normalizeAngle(goal_head_pos_map_rad_ - head_pos_map_rad);
			// ph.data = -head_pid_.update(e);
		}
		pepper_head_pub_->publish(ph);
		last_RX_status = abs(msg->axes[static_cast<int>(Axes::LT)]) > 0.1;
	}

	/*PID構造体をついか、元の操作量に足す
		最大値の設定をする*/
	// MARK: uodom cb
	void uodom_callback(const uec_msgs::msg::Odometry::SharedPtr msg)
	{
		yaw_ = msg->yaw;
	}

	// MARK: pepper_pos cb
	void pepper_pos_callback(const pepper_msgs::msg::PepperPos::SharedPtr msg)
	{
		head_pos_base_rad_ = (static_cast<float>(msg->pos_list[3]) - 512.0) * (2.0 * M_PI / 1023.0);
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
			rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
			rclcpp::Subscription<uec_msgs::msg::Odometry>::SharedPtr uodom_sub_;
			rclcpp::Subscription<pepper_msgs::msg::PepperPos>::SharedPtr pepper_pos_sub_;
		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pepper_head_pub_;

		float yaw_; // IMUのyaw角
		float head_pos_base_rad_ = 0.; // 頭のエンコーダの値(rad)
		float goal_head_pos_map_rad_ = 0.; // マップ座標系での目標頭部角(-pi~+pi rad)
		float goal_base_pos_map_rad_ = 0.; // マップ座標系での目標胴体角(-pi~+pi rad)

		PID head_pid_, base_pid_;
};

}  // namespace PEPPER_OMNI

// MARK: main
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PEPPER_OMNI::JoyPubNode>());
	rclcpp::shutdown();
	return 0;
}