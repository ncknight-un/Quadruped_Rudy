/// \file
/// \brief Creates a ROS 2 Node to command the quadruped to stand in a default static position by publishing joint angles to a topic.
/// The standing pose is the home position for the robot, and will be used as the default at start up.
/// Motors 1, 3, 5, 7 are abad joints, motors 2, 4, 6, 8 are hip joints, motors 9 and 10 are head joints, and motors 11, 13, 15, 17 are knee joints.
/// PARAMETERS:
///     ~ motor_ids (std::vector<double>, default: []) - List of motor IDs to command.
/// PUBLISHES:
///     ~ Joint States (sensor_msgs::msg::JointState): Publishes the joint states for the specified motor IDs to command the robot to stand in a static position.
/// SUBSCRIBES:
///     ~ None
/// SERVERS:
///     ~ stand (std_srvs::srv::Empty): Commands the robot to stand in a default static position.
///     ~ sit (std_srvs::srv::Empty): Commands the robot to sit in a default static position.
///     ~ kneel (std_srvs::srv::Empty): Commands the robot to kneel in a default static position.
///     ~ lie_down (std_srvs::srv::Empty): Commands the robot to lie down in a default static position.
/// CLIENTS:
///     ~ None

#include <cstdio>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <numbers>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_srvs/srv/empty.hpp"

// Control table address for X series
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// USB Connection Settings:
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

class StaticPositionNode : public rclcpp::Node {
public:
    StaticPositionNode() : Node("static_position_node")
    {
        RCLCPP_INFO(get_logger(), "Starting static position node...");

        // Verify that the motor ids and offsets are the same size:
        if (motor_ids_.size() != motor_offsets_.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "motor_ids and motor_calib_offsets must be same length!");
            throw std::runtime_error("Parameter size mismatch");
        }
        // Create a map to easily access motor offsets by ID:
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            motor_offset_map_[static_cast<int>(motor_ids_[i])] = motor_offsets_[i];
        }

        // Init Dynamixel Connection:
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Port and Enable Torque:
        for (const auto& motor_id: motor_ids_) {
            setupDynamixel(static_cast<uint8_t>(motor_id));
        }

        // Initialize the Services:
        // Stand Service:
        stand_service_ = this->create_service<std_srvs::srv::Empty>(
            "stand",
            std::bind(&StaticPositionNode::handle_service_stand, this, std::placeholders::_1, std::placeholders::_2)
        );
        // Sit Service:
        sit_service_ = this->create_service<std_srvs::srv::Empty>(
            "sit",
            std::bind(&StaticPositionNode::handle_service_sit, this, std::placeholders::_1, std::placeholders::_2)
        );
        // Kneel Service:
        kneel_service_ = this->create_service<std_srvs::srv::Empty>(
            "kneel",
            std::bind(&StaticPositionNode::handle_service_kneel, this, std::placeholders::_1, std::placeholders::_2)
        );
        // Lie Down Service:
        lie_down_service_ = this->create_service<std_srvs::srv::Empty>(
            "lie_down",
            std::bind(&StaticPositionNode::handle_service_lie_down, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Initialize the robot in the standing position at startup:
        RCLCPP_INFO(get_logger(), "Initializing robot in standing position...");
        std::vector<double> target_angles = standing_pose_;
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            command_motor_position(motor_ids_[i], target_angles[i]);
        }
    }

    private:
        // Motor encode ticks to radians:
        double ticks_to_radians(int32_t ticks) {
            return static_cast<double>(ticks) * (2 * std::numbers::pi) / eticks_per_rad_;
        }
        // Motor radians to encode ticks:
        int radians_to_ticks(double radians) {
            return static_cast<int32_t>(radians * eticks_per_rad_ / (2 * std::numbers::pi));
        }

        void setupDynamixel(uint8_t dxl_id) {
            int dxl_comm_result = COMM_TX_FAIL;
            uint8_t dxl_error = 0;

            // Use Position Control Mode
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_OPERATING_MODE,
                3,
                &dxl_error
            );

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Succeeded to set Position Control Mode.");
            }

            // Enable Torque of DYNAMIXEL
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                1,
                &dxl_error
            );

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to enable torque for motor ID " << static_cast<int>(dxl_id));

            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully enabled torque for motor ID %d.", dxl_id);
            }
        }

        // Motor Command Function:
        void command_motor_position(int motor_id, double target_angle) {
            // Determine the desired position in ticks and clamp to motor limits:
            int calibrated_target_pos = std::clamp(radians_to_ticks(target_angle) - motor_offset_map_[motor_id], 0, max_encoder_value_);

            // Send command to motor (example, adjust as needed):
            uint8_t dxl_error = 0;

            auto dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler,
                motor_id,
                ADDR_GOAL_POSITION,
                static_cast<uint32_t>(calibrated_target_pos),
                &dxl_error
            );
            if (dxl_error != 0) {
                RCLCPP_ERROR(get_logger(), "Motor error for ID %d: %d", motor_id, dxl_error);
            }

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", motor_id, calibrated_target_pos);
            }
        }

        // Service Handlers:
        void handle_service_stand(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received stand command, moving to static position...");

            // Define the target joint angles for standing position:
            std::vector<double> target_angles = standing_pose_;

            // Command the motors to move to the target angles:
            for (size_t i = 0; i < motor_ids_.size(); ++i) {
                command_motor_position(motor_ids_[i], target_angles[i]);
            }
        }
        void handle_service_sit(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received sit command, moving to static position...");

            // Define the target joint angles for sitting position:
            std::vector<double> target_angles = sitting_pose_;

            // Command the motors to move to the target angles:
            for (size_t i = 0; i < motor_ids_.size(); ++i) {
                command_motor_position(motor_ids_[i], target_angles[i]);
            }
        }
        void handle_service_kneel(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received kneel command, moving to static position...");

            // Define the target joint angles for kneeling position:
            std::vector<double> target_angles = kneeling_pose_;

            // Command the motors to move to the target angles:
            for (size_t i = 0; i < motor_ids_.size(); ++i) {
                command_motor_position(motor_ids_[i], target_angles[i]);
            }
        }
        void handle_service_lie_down(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received lie down command, moving to static position...");

            // Define the target joint angles for lying down position:
            std::vector<double> target_angles = lying_pose_;

            // Command the motors to move to the target angles:
            for (size_t i = 0; i < motor_ids_.size(); ++i) {
                command_motor_position(motor_ids_[i], target_angles[i]);
            }
        }

        // Initialize ROS 2 Services:
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stand_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sit_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr kneel_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr lie_down_service_;

        // Declare parameters
        // Initialization of motor id list:
        std::vector<int64_t>  motor_ids_ = declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});
        // Set the calibration offsets for each motor id:
        std::vector<int64_t> motor_offsets_ = this->declare_parameter<std::vector<int64_t>>("motor_calib_offsets", std::vector<int64_t>{});
        // Get the encode_ticks_per_rad parameter:
        int eticks_per_rad_ = this->declare_parameter<int>("encode_ticks_per_rad", 652);
        // Set the max encoder value parameter:
        int max_encoder_value_ = this->declare_parameter<int>("max_encoder_value", 4096);

        // Initialize Motor Offset Map:
        std::unordered_map<int, int> motor_offset_map_;

        // Get the target joint angles for each pose:
        std::vector<double> standing_pose_ = this->declare_parameter<std::vector<double>>("standing_pose", std::vector<double>{});
        std::vector<double> sitting_pose_ = this->declare_parameter<std::vector<double>>("sitting_pose", std::vector<double>{});
        std::vector<double> kneeling_pose_ = this->declare_parameter<std::vector<double>>("kneeling_pose", std::vector<double>{});
        std::vector<double> lying_pose_ = this->declare_parameter<std::vector<double>>("lying_pose", std::vector<double>{});

        // Initializzation of Port Handlers:
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticPositionNode>());
  rclcpp::shutdown();
  return 0;
}
