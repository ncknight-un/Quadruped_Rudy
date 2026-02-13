/// \file
/// \brief Creates a ROS 2 Node to calibrate the quadruped's motors using Dynamixel SDK, then terminates the node and saves encoder readings to a YAML File.
/// PARAMETERS:
///     ~ motor_ids (std::vector<double>, default: []) - List of motor IDs to calibrate.
/// PUBLISHES:
///     ~ None
/// SUBSCRIBES:
///     ~ None
/// SERVERS:
///     ~ None
/// CLIENTS:
///     ~ None

#include <cstdio>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

// Control table address for X series
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// USB Connection Settings:
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"


using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

class CalibrateNode : public rclcpp::Node {
public:
  CalibrateNode() : Node("calibration_node")
  {
    RCLCPP_INFO(get_logger(), "Starting calibration...");

    // Declare parameters
    motor_ids_ = declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});

    // Init Dynamixel Connection:
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Port and Set Baudrate successfully:
    if (!portHandler->openPort()) {
      RCLCPP_FATAL(get_logger(), "Failed to open port");
      rclcpp::shutdown();
      return;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
      RCLCPP_FATAL(get_logger(), "Failed to set baudrate");
      rclcpp::shutdown();
      return;
    }

    calibrate();
  }

private:
  // Function to perform calibration by reading current positions of specified motors and saving to YAML:
  void calibrate()
  {
    YAML::Node output;

    for (auto id : motor_ids_) {
      int32_t encoder_position = read_position(static_cast<uint8_t>(id));
      output["motors"][std::to_string(id)] = encoder_position;
    }

    std::ofstream fout("motor_calibration.yaml");
    fout << output;
    fout.close();

    RCLCPP_INFO(get_logger(), "Calibration saved.");
    
    // Close the port after calibration:
    portHandler->closePort();
    // Shut down the node after calibration:
    rclcpp::shutdown();
  }

  // Function to read the current position of a motor by ID:
  int32_t read_position(uint8_t id)
  {
    // Init position and error:
    int32_t present_position = 0;
    uint8_t dxl_error = 0;

    auto dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      id,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position),
      &dxl_error
    );

    // Error handling for Motor Read Issues:
    if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Read failed for ID %d: %s", id,
                 packetHandler->getTxRxResult(dxl_comm_result));
    }
    if (dxl_error != 0) {
        RCLCPP_ERROR(get_logger(), "Motor error for ID %d: %d", id, dxl_error);
    }

    return present_position;
  }

  // Initialization of motor id list:
  std::vector<int64_t> motor_ids_;

  // Initializzation of Port Handlers:
  dynamixel::PortHandler * portHandler;
  dynamixel::PacketHandler * packetHandler;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalibrateNode>();
  return 0;
}
