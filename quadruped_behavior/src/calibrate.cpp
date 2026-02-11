#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table address for X series
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

class CalibrationNode : public rclcpp::Node {
public: 
	CalibrationNode(): Node("calibration_node")
	{
		RCLCPP_INFO(this->get_logge(), "Starting Calibration...");
		
		// Initialize Dynamixel SDK:
		portHandler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::makeshared<CalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
