#include <chrono>
#include <memory>
#include <string>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include "pepper_msgs/msg/pepper_pos.hpp"
#include <cmath>
#include "uec_utils/uec_utils.hpp"

using namespace std::chrono_literals;

namespace PEPPER_OMNI {

// forward declarations
int32_t read_present_position(uint8_t dxl_id);

// Control table (Protocol 1.0 / MX & RX legacy addresses)
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_MOVING_SPEED 32
#define ADDR_PRESENT_POSITION 36

// Dynamixel communication configuration
#define PROTOCOL_VERSION 1.0
#define BAUDRATE 115200
#define DEVICE_NAME "/dev/ttyUSB0"

// Forward declare shared Dynamixel SDK objects so class methods can use them
extern dynamixel::PortHandler * portHandler;
extern dynamixel::PacketHandler * packetHandler;
extern uint8_t dxl_error;
extern int dxl_comm_result;

class DxlOmniNode : public rclcpp::Node
{
public:
  // MARK: constructor
  DxlOmniNode(): Node("dxl_omni")
  {
    declare_parameter("id list", std::vector<int64_t>{1, 2, 3, 4});
    id_list_ = get_parameter("id list").as_integer_array();
    declare_parameter("hz", 100.0); // default 100 Hz
    hz_ = this->get_parameter("hz").as_double();
    if (hz_ <= 0.0) {
      hz_ = 100;
    }

    // Subscribe to cmd_vel (Twist)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&DxlOmniNode::twist_cb, this, std::placeholders::_1)
    );

      // Subscribe to pepper_head (double)
    pepper_head_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/pepper_head",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&DxlOmniNode::pepper_head_cb, this, std::placeholders::_1)
    );

      // Publisher for pepper positions
      pepper_pos_pub_ = this->create_publisher<pepper_msgs::msg::PepperPos>(
        "/pepper_pos",
        rclcpp::QoS(rclcpp::KeepLast(10)));
    
    // compute period from Hz and create timer
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / hz_));
    timer_ = this->create_wall_timer(period, std::bind(&DxlOmniNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "dxl_omni node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<int64_t> id_list_;
  double hz_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pepper_head_sub_;
  rclcpp::Publisher<pepper_msgs::msg::PepperPos>::SharedPtr pepper_pos_pub_;
  double latest_pepper_head_ = 0.0;
  double last_sent_pepper_head_ = 0.0;

  // MARK: twist cb
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float u[3], v, du;
    static float prev_u_[3] = {0.0, 0.0, 0.0};
    constexpr float wheel_angle_[3] = {
      M_PI_2,
      M_PI + M_PI / 6.0,
      -M_PI / 6.0
    };

    u[0] = msg->linear.y;
    u[1] = -msg->linear.x;
    u[2] = 0.118 * msg->angular.z;
    for (int i = 0; i < 3; ++i) {
        du = u[i] - prev_u_[i];
        u[i] = DELTA_LIMIT(u[i], prev_u_[i], du, 1.);
        prev_u_[i] = u[i];
    }

    float data[3];
    for (int i = 0; i < 3; ++i) {
        v = u[0] * cosf(wheel_angle_[i]) +
            u[1] * sinf(wheel_angle_[i]) - u[2];
        // 最大値制限
        v = MAX_LIMIT(v, 1.0);
        // 最小値制限
        v = MIN_LIMIT(v, -1.0);
        data[i] = v *(1.0/0.05);

        send_velocity_command(static_cast<uint8_t>(i + 1), data[i]);
    }
  }

  // Read a single servo position via namespace helper
  int32_t read_servo_position(uint8_t id)
  {
    int32_t pos = PEPPER_OMNI::read_present_position(id);
    if (pos == INT32_MIN) {
      RCLCPP_WARN(this->get_logger(), "read_servo_position: failed to read ID=%d", id);
    } 
    // else {
    //   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "read_servo_position: ID=%d pos=%d", id, pos);
    // }
    return pos;
  }

  // MARK: send vel
  // Send a velocity (moving speed) command to an arbitrary servo ID (wheel mode)
  // speed_rad_s: desired angular speed in [rad/s]. Function converts to
  // Dynamixel Protocol 1.0 Moving Speed register units for wheel mode.
  // Wheel mode format: 0..1023 = magnitude (CCW), 1024..2047 = magnitude + direction(CW)
  // NOTE: unit -> ~0.114 rpm per register unit for many MX/RX models.
  bool send_velocity_command(uint8_t id, double speed_rad_s, double rpm_per_unit = 0.114)
  {
    const double factor = 60.0 / (2.0 * M_PI * rpm_per_unit);

    double abs_units = std::round(std::abs(speed_rad_s) * factor);
    // wheel-mode only: magnitude fits in 0..1023, direction in bit 10 (1024)
    if (abs_units > 1023.0) abs_units = 1023.0;
    uint16_t magnitude = static_cast<uint16_t>(abs_units);
    uint16_t raw_speed = magnitude;
    // Encode direction: per e-manual, 0..1023 => CCW, 1024..2047 => CW
    if (speed_rad_s > 0.0) {
      raw_speed |= 0x400; // set bit 10 to indicate CW
    }

    int result = PEPPER_OMNI::packetHandler->write2ByteTxRx(
      PEPPER_OMNI::portHandler,
      id,
      ADDR_MOVING_SPEED,
      raw_speed,
      &PEPPER_OMNI::dxl_error
    );

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "send_velocity_command: write failed ID=%d: %s", id, PEPPER_OMNI::packetHandler->getTxRxResult(result));
      return false;
    }
    if (PEPPER_OMNI::dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "send_velocity_command: servo error ID=%d: %s", id, PEPPER_OMNI::packetHandler->getRxPacketError(PEPPER_OMNI::dxl_error));
      return false;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "send_velocity_command: ID=%d speed=%.3frad/s -> raw=%u", id, speed_rad_s, raw_speed);
    return true;
  }

  // MARK: timer cb
  void timer_cb()
  {
    // Read all servo positions and publish as PepperPos
    pepper_msgs::msg::PepperPos out_msg;
    out_msg.pos_list.clear();
    for (auto id64 : id_list_) {
      uint8_t id = static_cast<uint8_t>(id64);
      int32_t p = read_servo_position(id);
      out_msg.pos_list.push_back(static_cast<int64_t>(p));
    }
    pepper_pos_pub_->publish(out_msg);
  }

  // MARK: head cb
  void pepper_head_cb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // store and log the received value
    latest_pepper_head_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "pepper_head received: %.3f", latest_pepper_head_);
    // Only send if value changed significantly to avoid jitter
    const double eps = 1e-3;
    if (std::abs(latest_pepper_head_ - last_sent_pepper_head_) > eps) {
      bool ok = send_velocity_command(4, latest_pepper_head_, 0.111);
      if (ok) last_sent_pepper_head_ = latest_pepper_head_;
      else RCLCPP_WARN(this->get_logger(), "send_velocity_command failed for head value %.3f", latest_pepper_head_);
    }
  }
};

} // namespace PEPPER_OMNI

// Dynamixel SDK objects and control settings for Protocol 1.0
namespace PEPPER_OMNI {

dynamixel::PortHandler * portHandler = nullptr;
dynamixel::PacketHandler * packetHandler = nullptr;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

// MARK: init dxl
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

/* MARK: read dxl pos
  Read present position (Protocol 1.0: 2-byte position for MX/RX/AX)
 */
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

// MARK: main
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
