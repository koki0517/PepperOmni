#include <chrono>
#include <memory>
#include <string>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace PEPPER_OMNI {

// forward declarations
int32_t read_present_position(uint8_t dxl_id);

class DxlOmniNode : public rclcpp::Node
{
public:
  DxlOmniNode()
  : Node("dxl_omni")
  {
    declare_parameter("id list", std::vector<int64_t>{1, 2, 3});
    id_list_ = get_parameter("id list").as_integer_array();
    declare_parameter("hz", 100); // default 100 Hz
    hz_ = this->get_parameter("hz").as_double();
    if (hz_ <= 0.0) {
      hz_ = 100;
    }

    // Subscribe to cmd_vel (Twist)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&DxlOmniNode::cmd_vel_callback, this, std::placeholders::_1)
    );
    
    // compute period from Hz and create timer
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / hz_));
    timer_ = this->create_wall_timer(period, std::bind(&DxlOmniNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "dxl_omni node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<int64_t> id_list_;
  double hz_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // member callback for cmd_vel
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "cmd_vel (member): linear=(%.3f, %.3f, %.3f) angular=(%.3f, %.3f, %.3f)",
      msg->linear.x, msg->linear.y, msg->linear.z,
      msg->angular.x, msg->angular.y, msg->angular.z);

    // TODO: translate Twist -> Dynamixel commands
  }

  // Read a single servo position via namespace helper
  int32_t read_servo_position(uint8_t id)
  {
    int32_t pos = PEPPER_OMNI::read_present_position(id);
    if (pos == INT32_MIN) {
      RCLCPP_WARN(this->get_logger(), "read_servo_position: failed to read ID=%d", id);
    } else {
      RCLCPP_INFO(this->get_logger(), "read_servo_position: ID=%d pos=%d", id, pos);
    }
    return pos;
  }

  // Read positions for all configured ids
  void timer_callback()
  {
    for (auto id64 : id_list_) {
      uint8_t id = static_cast<uint8_t>(id64);
      read_servo_position(id);
    }
  }
};

} // namespace PEPPER_OMNI

// Dynamixel SDK objects and control settings for Protocol 1.0
namespace PEPPER_OMNI {

dynamixel::PortHandler * portHandler = nullptr;
dynamixel::PacketHandler * packetHandler = nullptr;

// Control table (Protocol 1.0 / MX & RX legacy addresses)
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

#define PROTOCOL_VERSION 1.0
#define BAUDRATE 1000000
#define DEVICE_NAME "/dev/ttyUSB0"

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

void setup_dynamixel(uint8_t dxl_id)
{
  // For Protocol 1.0 (MX, RX legacy), simply enable torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dxl_omni"), "Failed to enable torque (ID=%d): %s", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dxl_omni"), "Error enabling torque (ID=%d): %s", dxl_id, packetHandler->getRxPacketError(dxl_error));
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dxl_omni"), "Succeeded to enable torque (ID=%d)", dxl_id);
  }
}

// Read present position (Protocol 1.0: 2-byte position for MX/RX/AX)
int32_t read_present_position(uint8_t dxl_id)
{
  uint16_t present_position = 0;
  PEPPER_OMNI::dxl_comm_result = PEPPER_OMNI::packetHandler->read2ByteTxRx(
    PEPPER_OMNI::portHandler,
    dxl_id,
    ADDR_PRESENT_POSITION,
    &present_position,
    &PEPPER_OMNI::dxl_error
  );

  if (PEPPER_OMNI::dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dxl_omni"), "Failed to read position (ID=%d): %s", dxl_id, PEPPER_OMNI::packetHandler->getTxRxResult(PEPPER_OMNI::dxl_comm_result));
    return INT32_MIN;
  }

  if (PEPPER_OMNI::dxl_error != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dxl_omni"), "Error reading position (ID=%d): %s", dxl_id, PEPPER_OMNI::packetHandler->getRxPacketError(PEPPER_OMNI::dxl_error));
    return INT32_MIN;
  }

  return static_cast<int32_t>(present_position);
}
} // namespace PEPPER_OMNI


int main(int argc, char ** argv)
{
  // Initialize Dynamixel SDK port and packet handlers (Protocol 1.0)
  PEPPER_OMNI::portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  PEPPER_OMNI::packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open serial port
  PEPPER_OMNI::dxl_comm_result = PEPPER_OMNI::portHandler->openPort();
  if (PEPPER_OMNI::dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("dxl_omni"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dxl_omni"), "Succeeded to open the port.");
  }

  // Set the baudrate for the port
  PEPPER_OMNI::dxl_comm_result = PEPPER_OMNI::portHandler->setBaudRate(BAUDRATE);
  if (PEPPER_OMNI::dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("dxl_omni"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dxl_omni"), "Succeeded to set the baudrate.");
  }

  // Setup devices: MX-64 ids 1..3 and RX-64F id 4
  PEPPER_OMNI::setup_dynamixel(1);
  PEPPER_OMNI::setup_dynamixel(2);
  PEPPER_OMNI::setup_dynamixel(3);
  PEPPER_OMNI::setup_dynamixel(4);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PEPPER_OMNI::DxlOmniNode>());
  rclcpp::shutdown();
  // Disable torque on the configured motors (ids 1-4)
  PEPPER_OMNI::packetHandler->write1ByteTxRx(PEPPER_OMNI::portHandler, 1, ADDR_TORQUE_ENABLE, 0, &PEPPER_OMNI::dxl_error);
  PEPPER_OMNI::packetHandler->write1ByteTxRx(PEPPER_OMNI::portHandler, 2, ADDR_TORQUE_ENABLE, 0, &PEPPER_OMNI::dxl_error);
  PEPPER_OMNI::packetHandler->write1ByteTxRx(PEPPER_OMNI::portHandler, 3, ADDR_TORQUE_ENABLE, 0, &PEPPER_OMNI::dxl_error);
  PEPPER_OMNI::packetHandler->write1ByteTxRx(PEPPER_OMNI::portHandler, 4, ADDR_TORQUE_ENABLE, 0, &PEPPER_OMNI::dxl_error);

  return 0;
}
