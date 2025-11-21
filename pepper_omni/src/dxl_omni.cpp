#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class DxlOmniNode : public rclcpp::Node
{
public:
  DxlOmniNode()
  : Node("dxl_omni")
  {
    RCLCPP_INFO(this->get_logger(), "dxl_omni node started");
    timer_ = this->create_wall_timer(1s, [this]() {
      RCLCPP_INFO(this->get_logger(), "hello from pepper_omni dxl_omni node");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlOmniNode>());
  rclcpp::shutdown();
  return 0;
}
