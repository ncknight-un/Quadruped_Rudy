/// \file
/// \brief Creates a ROS 2 Node to command the quadruped to stand in a default static position by publishing joint angles to a topic.
/// The standing pose is the home position for the robot, and will be used as the default at start up.
/// Motors 1, 3, 5, 7 are abad joints, motors 2, 4, 6, 8 are hip joints, motors 9 and 10 are head joints, and motors 11, 13, 15, 17 are knee joints.
/// PARAMETERS:
///     ~ rate (double, default: 100.0) - The frequency at which to publish joint states and motor commands.
//      ~ motor_ids (std::vector<double>, default: []) - List of motor IDs to command.
/// PUBLISHES:
///     ~ Joint States (sensor_msgs::msg::JointState): Publishes the joint states for the specified motor IDs to command the robot to stand in a static position.
///     ~/timestep (std_msgs::msg::UInt64): Publishes the current timestep of the node at the specified rate.
///     ~ tf (geometry_msgs::msg::TransformStamped): Publishes the tf transform from the robot base to the world frame. (Will implement in future)
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
#include "std_msgs/msg/u_int64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

// Control table address for X series
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY     112

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// USB Connection Settings:
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Create a State Controller:
enum class RobotState {
    STANDING,
    SITTING,
    KNEELING,
    LYING,
    WAITING
};

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
        if (!portHandler->openPort()) {
            RCLCPP_FATAL(get_logger(), "Failed to open port %s", DEVICE_NAME);
            throw std::runtime_error("Failed to open port");
        }
        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_FATAL(get_logger(), "Failed to set baudrate %d", BAUDRATE);
            throw std::runtime_error("Failed to set baudrate");
        }
        // Wait for Connection: 
        rclcpp::sleep_for(std::chrono::seconds(2));
        for (const auto& motor_id: motor_ids_) {
            // Small delay between motor setups (avoids bus overload)
            rclcpp::sleep_for(std::chrono::milliseconds(50));
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

        // Initialize the Publisher:
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        // Initialize the ~/timestep Publisher:
        timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        
        auto timer_callback = [this]() -> void {

            // Print Once:
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "The Timer rate is " << rate_ << " Hz!");

            // Publish the timestep:
            std_msgs::msg::UInt64 message;
            message.data = timestep_++;
            timestep_publisher_->publish(message);

            // Read Raw Encode Ticks from Motors:
            // NOTE: Offset is applied in ReadAllMotorPosition Function.
            std::vector<int32_t> calibrated_ticks = ReadAllMotorPosition();

            if (calibrated_ticks.size() != motor_ids_.size()) {
                RCLCPP_ERROR(this->get_logger(), "ReadAllMotorPosition returned wrong size!");
                return;
            }

            // Convert encoder ticks -> joint radians
            std::vector<double> current_joints(motor_ids_.size());

            for (size_t i = 0; i < motor_ids_.size(); ++i) {
                // Convert to radians for current joint position:
                current_joints[i] = ticks_to_radians(calibrated_ticks[i]);  // At start up should be 0.
                if(current_state_ == RobotState::STANDING) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Current Motor ID: " << motor_ids_[i] << " | Calibrated Ticks: " << calibrated_ticks[i] << " | Current Angle (rad): " << current_joints[i]);
                }
            }

            // Determine target joint angles (radians)
            std::vector<double> target_joints;

            switch (current_state_)
            {
                case RobotState::STANDING:
                    target_joints = standing_pose_;
                    break;

                case RobotState::SITTING:
                    target_joints = sitting_pose_;
                    break;

                case RobotState::KNEELING:
                    target_joints = kneeling_pose_;
                    break;

                case RobotState::LYING:
                    target_joints = lying_pose_;
                    break;

                case RobotState::WAITING:
                default:
                    break;
            }

            // // 4. Publish measured joint states
            // sensor_msgs::msg::JointState joint_state_msg;
            // joint_state_msg.header.stamp = this->now();
            // joint_state_msg.name.resize(motor_ids_.size());
            // joint_state_msg.position = current_joints;

            // for (size_t i = 0; i < motor_ids_.size(); ++i) {
            //     joint_state_msg.name[i] = "motor_" + std::to_string(motor_ids_[i]);
            // }

            // joint_state_pub_->publish(joint_state_msg);

            // Send motor commands if not waiting
            if (current_state_ != RobotState::WAITING) {
                if (target_joints.size() != motor_ids_.size()) {
                    RCLCPP_ERROR(this->get_logger(),
                                "Target pose size does not match motor count!");
                    return;
                }

                for (size_t i = 0; i < motor_ids_.size(); ++i)
                {
                    command_motor_position(motor_ids_[i], target_joints[i]);
                }
            }
        };

        // Initialize the timer: 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / rate_)), timer_callback);

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

            // Disable torque at shutdown:
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to disable torque for motor " << int(dxl_id));
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "Torque disabled for motor " << int(dxl_id));
            }

            // Set operating mode to Position Control
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 3, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set Position Control Mode for motor " << int(dxl_id));
            }

            // Enable torque
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to enable torque for motor " << int(dxl_id));
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "Motor " << int(dxl_id) << " ready in Position Control Mode.");
            }

            // Turn on Acceleration and Velocity Profiles:
            // Set Profile Acceleration
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_PROFILE_ACCELERATION,
                profile_acceleration_,
                &dxl_error
            );

            // Set Profile Velocity
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_PROFILE_VELOCITY,
                profile_velocity_,
                &dxl_error
            );

            rclcpp::sleep_for(std::chrono::milliseconds(50)); // Small delay after setup
        }

        // Get current Motor Positions:
        std::vector<int32_t> ReadAllMotorPosition(){
            std::vector<int32_t> positions;
            for (const auto& motor_id: motor_ids_) {
                int32_t present_position = 0;
                uint8_t dxl_error = 0;

                auto dxl_comm_result = packetHandler->read4ByteTxRx(
                    portHandler,
                    static_cast<uint8_t>(motor_id),
                    ADDR_PRESENT_POSITION,
                    reinterpret_cast<uint32_t*>(&present_position),
                    &dxl_error
                );

                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read position for motor " << motor_id);
                    positions.push_back(0.0); // Default to 0 on error
                } else {
                    double position_rad = ticks_to_radians(present_position - motor_offset_map_[motor_id]);
                    positions.push_back(position_rad);
                }
            }
            return positions;
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

            // Set the State:
            current_state_ = RobotState::STANDING;
        }
        void handle_service_sit(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received sit command, moving to static position...");

            // Set the State:
            current_state_ = RobotState::SITTING;
        }
        void handle_service_kneel(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received kneel command, moving to static position...");

            // Set the State:
            current_state_ = RobotState::KNEELING;
        }
        void handle_service_lie_down(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received lie down command, moving to static position...");

            // Set the State:
            current_state_ = RobotState::LYING;
        }

        // Initialize ROS 2 Services:
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stand_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sit_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr kneel_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr lie_down_service_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;

        // Initialize the timer:
        rclcpp::TimerBase::SharedPtr timer_;

        // Declare parameters
        // Initialization of motor id list:
        std::vector<int64_t>  motor_ids_ = declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});
        // Set the calibration offsets for each motor id:
        std::vector<int64_t> motor_offsets_ = this->declare_parameter<std::vector<int64_t>>("motor_calib_offsets", std::vector<int64_t>{});
        // Get the encode_ticks_per_rad parameter:
        int eticks_per_rad_ = this->declare_parameter<int>("encode_ticks_per_rad", 652);
        // Set the max encoder value parameter:
        int max_encoder_value_ = this->declare_parameter<int>("max_encoder_value", 4096);
        // Set the profile velocity and acceleration parameters:
        int profile_velocity_ = this->declare_parameter<int>("profile_velocity", 80);
        int profile_acceleration_ = this->declare_parameter<int>("profile_acceleration", 30);

        // Set the timer frequency:
        double rate_ = this->declare_parameter<double>("frequency", 100.0);
        // Initialize a timestep counter for the timer:
        uint64_t timestep_ = 0;

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

        // Initialize State Controller:
        RobotState current_state_ = RobotState::WAITING;

        // Track Current Motor Positions:
        std::vector<double> current_motor_positions_ = {};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticPositionNode>());
  rclcpp::shutdown();
  return 0;
}
