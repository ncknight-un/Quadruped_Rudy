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
///     ~ give_paw (std_srvs::srv::Empty): Commans the robot to give a paw in a default static position.
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
    GIVE_PAW,
    WALKING,
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
        // Give Paw Service:
        give_paw_service_ = this->create_service<std_srvs::srv::Empty>(
            "give_paw",
            std::bind(&StaticPositionNode::handle_service_give_paw, this, std::placeholders::_1, std::placeholders::_2)
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
        // Lie Down Service:
        lie_down_service_ = this->create_service<std_srvs::srv::Empty>(
            "walk",
            std::bind(&StaticPositionNode::handle_service_walk, this, std::placeholders::_1, std::placeholders::_2)
        );
        // Reset Torque Service:
        reset_torque_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_torque",
            std::bind(&StaticPositionNode::handle_service_reset_torque, this, std::placeholders::_1, std::placeholders::_2)
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
            std::vector<int32_t> calibrated_ticks = ReadAllMotorPosition();

            // If this is first loop, set the last_motor_ticks_ to the current position to avoid large jumps on first command:
            if (last_motor_ticks_.empty()) {
                for (size_t i = 0; i < motor_ids_.size(); ++i) {
                    last_motor_ticks_[motor_ids_[i]] = calibrated_ticks[i];
                }
            }

            if (calibrated_ticks.size() != motor_ids_.size()) {
                RCLCPP_ERROR(this->get_logger(), "ReadAllMotorPosition returned wrong size!");
                return;
            }

            // Convert encoder ticks -> joint radians
            std::vector<double> current_joints(motor_ids_.size());

            for (size_t i = 0; i < motor_ids_.size(); ++i) {
                // Convert to radians for current joint position:
                current_joints[i] = ticks_to_radians(calibrated_ticks[i]);
            }

            // Determine target joint angles (radians)
            std::vector<double> target_joints;

            switch (current_state_) {
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
                
                case RobotState::GIVE_PAW:
                    target_joints = paw_pose_;
                    break;
                case RobotState::WALKING:
                    // Set the target_joints to standing (to set all other joints, then in loop adjust one leg at a time)
                    target_joints = standing_pose_;
                    break;
                case RobotState::WAITING:
                default:
                    break;
            }
            
            // TODO: Map these to actual joints and publish:
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
                    RCLCPP_ERROR(this->get_logger(), "Target pose size does not match motor count!");
                    return;
                }
                
                // If state has been changed, print the target joint angles once:
                static RobotState last_state = RobotState::WAITING;
                if (current_state_ != last_state && current_state_ != RobotState::WALKING) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "New State: " << (current_state_ == RobotState::STANDING ? "STANDING" :
                                                        (current_state_ == RobotState::SITTING ? "SITTING" :
                                                        (current_state_ == RobotState::KNEELING ? "KNEELING" :
                                                        (current_state_ == RobotState::GIVE_PAW ? "GIVE_PAW" :
                                                        (current_state_ == RobotState::LYING ? "LYING" : "WAITING"))))));

                    // Command the robot to the static pose:
                    for (size_t i = 0; i < motor_ids_.size(); ++i) {
                        RCLCPP_DEBUG_STREAM(this->get_logger(), "Target Motor ID: " << motor_ids_[i] << " | Target Angle (rad): " << target_joints[i]);
                        command_motor_position(motor_ids_[i], target_joints[i]);
                        rclcpp::sleep_for(std::chrono::milliseconds(5)); // Small delay to avoid overloading the bus.
                    }
                    last_state = current_state_;

                    rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small delay after commanding new state before next timer callback.
                }
                else if(current_state_ == RobotState::WALKING) {
                    // If the state is walking, loop through the walking pose sequence:
                    RCLCPP_INFO_STREAM(this->get_logger(), "New State: WALKING");

                    // Initialize the walk sequence at static standing: 
                    if(current_state_ != last_state) {
                        for (size_t i = 0; i < motor_ids_.size(); ++i) {
                            RCLCPP_DEBUG_STREAM(this->get_logger(), "Target Motor ID: " << motor_ids_[i] << " | Target Angle (rad): " << target_joints[i]);
                            command_motor_position(motor_ids_[i], target_joints[i]);
                            rclcpp::sleep_for(std::chrono::milliseconds(5)); // Small delay to avoid overloading the bus.
                        }
                    }

                    // Define motor ids for each leg
                    std::vector<std::vector<int64_t>> legs = {
                        {motor_ids_[0],  motor_ids_[1],  motor_ids_[2]},   // BL
                        {motor_ids_[3],  motor_ids_[4],  motor_ids_[5]},   // BR
                        {motor_ids_[6],  motor_ids_[7],  motor_ids_[8]},   // FL
                        {motor_ids_[9],  motor_ids_[10], motor_ids_[11]}   // FR
                    };

                    int joints_per_leg = 3;
                    int num_phases = walking_pose_sequence_.size() / joints_per_leg;
                    if (num_phases == 0) return;

                    // Extract pose for current phase
                    std::vector<double> pose(
                        walking_pose_sequence_.begin() + walking_phase_ * joints_per_leg,
                        walking_pose_sequence_.begin() + (walking_phase_ + 1) * joints_per_leg
                    );

                    // Send pose to each leg
                    for (size_t leg = 0; leg < legs.size(); ++leg) {
                        const auto& motor_ids = legs[leg];

                        if (pose.size() != motor_ids.size()) {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Pose size does not match motor count for leg " << leg);
                            continue;
                        }

                        for (size_t j = 0; j < motor_ids.size(); ++j) {
                            command_motor_position(motor_ids[j], pose[j]);
                        }
                    }

                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                    last_state = current_state_;

                    walking_phase_ = (walking_phase_ + 1) % num_phases; // Loop through the walking phases
                }
            }
        };

        // Initialize the timer: 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / rate_)), timer_callback);
    }

    private:
        // Motor encode ticks to radians:
        double ticks_to_radians(int32_t ticks) {
            return static_cast<double>(ticks) / eticks_per_rad_;
        }
        // Motor radians to encode ticks:
        int radians_to_ticks(double radians) {
            return static_cast<int32_t>(radians * eticks_per_rad_);
        }
        // ############################### Begin_Citation [Angle Normalization Function] ###############################
        // Function to wrap target angle from 0 to 2Pi:
        constexpr double normalize_angle_0_2pi(double rad)
        {
            using std::numbers::pi;
            rad = std::fmod(rad, 2.0 * pi);  // wrap to [-2*pi, 2*pi]

            if (rad < 0.0) {
                rad += 2.0 * pi;            // shift negative angles into [0, 2*pi)
            }

            return rad;
        }
        // ############################### End_Citation [Angle Normalization Function] ###############################

        void setupDynamixel(uint8_t dxl_id) {
            int dxl_comm_result = COMM_TX_FAIL;
            uint8_t dxl_error = 0;

            // Disable torque
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Comm failed while disabling torque for motor " << int(dxl_id));
                return;
            }

            RCLCPP_INFO_STREAM(this->get_logger(),
                "Torque disabled for motor " << int(dxl_id));

            // Allow hardware to settle (critical for XC430)
            rclcpp::sleep_for(std::chrono::milliseconds(20));

            // Set operating mode (Position Control = 3)
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, dxl_id, ADDR_OPERATING_MODE, 3, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Comm failed while setting mode for motor " << int(dxl_id));
                return;
            }

            // Verify mode by reading it back
            uint8_t mode_read = 0;
            dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, dxl_id, ADDR_OPERATING_MODE, &mode_read, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS || mode_read != 3) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Mode NOT set correctly for motor " << int(dxl_id)
                    << " (read: " << int(mode_read) << ")");
                return;
            }

            RCLCPP_INFO_STREAM(this->get_logger(),
                "Motor " << int(dxl_id) << " in Position Control Mode");

            // Small delay before re-enabling torque
            rclcpp::sleep_for(std::chrono::milliseconds(10));

            // Enable torque
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                    "Failed to enable torque for motor " << int(dxl_id));
                return;
            }

            // Set motion profiles (non-critical → don't fail hard)
            packetHandler->write4ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_PROFILE_ACCELERATION,
                profile_acceleration_,
                &dxl_error
            );

            packetHandler->write4ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_PROFILE_VELOCITY,
                profile_velocity_,
                &dxl_error
            );

            RCLCPP_INFO_STREAM(this->get_logger(),
                "Motor " << int(dxl_id) << " ready.");

            // Final small delay
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }

        // Get current Motor Positions:
        // Returns the encoder position of each motor in radians.
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
                    positions.push_back(0); // Default to 0 on error
                } else {
                    auto position_rad = present_position; // motor_offset_map_[motor_id];
                    positions.push_back(position_rad);
                }
            }
            return positions;
        }

        // Motor Command Function:
        void command_motor_position(int motor_id, double target_angle_rad) {
            // Convert target angle to encoder ticks
            int32_t target_ticks = radians_to_ticks(normalize_angle_0_2pi(target_angle_rad));

            // Apply calibration offset
            target_ticks += motor_offset_map_[motor_id];

            // Read current motor position
            uint32_t current_ticks = 0;
            uint8_t dxl_error = 0;
            auto dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler,
                static_cast<uint8_t>(motor_id),
                ADDR_PRESENT_POSITION,
                &current_ticks,
                &dxl_error
            );
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read current position for motor " << motor_id);
                current_ticks = last_motor_ticks_[motor_id]; // Use last known position on read error
            }

            // Compute delta in circular encoder space
            int32_t delta = static_cast<int32_t>(target_ticks) - static_cast<int32_t>(current_ticks);
            int32_t half_range = (max_encoder_value_ + 1) / 2;

            // Wrap delta to [-half_range, +half_range]
            if (delta > half_range) delta -= (max_encoder_value_ + 1);
            if (delta < -half_range) delta += (max_encoder_value_ + 1);

            // Compute final target and wrap it into [0, max_encoder_value_]
            int32_t final_target = (current_ticks + delta + max_encoder_value_ + 1) % (max_encoder_value_ + 1);

            RCLCPP_INFO_STREAM(this->get_logger(), "Commanding Motor ID: " << motor_id
                << " | Target Angle (rad): " << target_angle_rad
                << " | Current Ticks: " << current_ticks
                << " | Target Position (ticks): " << final_target);

            // Send the final target to motor
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler,
                static_cast<uint8_t>(motor_id),
                ADDR_GOAL_POSITION,
                static_cast<uint32_t>(final_target),
                &dxl_error
            );

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "TX/RX Error: " << packetHandler->getTxRxResult(dxl_comm_result));
            }
            if (dxl_error != 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Motor error: " << packetHandler->getRxPacketError(dxl_error));
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

            // Set the State:9
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
        void handle_service_give_paw(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received give paw command, moving to static position...");

            // Set the State:
            current_state_ = RobotState::GIVE_PAW;
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
        void handle_service_walk(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received walk command, moving to static position...");

            // Set the State:
            current_state_ = RobotState::WALKING;
        }
        // Reset Torque Handler:
        void handle_service_reset_torque(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response) {
            // Ignore Request and Response:
            (void)request;
            (void)response;

            RCLCPP_INFO(get_logger(), "Received reset torque command, disabling torque on all motors...");
            // Disable torque on all motors:
            for (const auto& motor_id: motor_ids_) {
                uint8_t dxl_error = 0;
                auto dxl_comm_result = packetHandler->write1ByteTxRx(
                    portHandler,
                    static_cast<uint8_t>(motor_id),
                    ADDR_TORQUE_ENABLE,
                    0,
                    &dxl_error
                );
                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to disable torque for motor " << motor_id);
                } else {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Torque disabled for motor " << motor_id);
                }
                rclcpp::sleep_for(std::chrono::milliseconds(50)); // Small delay between motor commands
            }
            // Turn Torque back on after reset and set state to WAITING:
            for (const auto& motor_id: motor_ids_) {
                uint8_t dxl_error = 0;
                auto dxl_comm_result = packetHandler->write1ByteTxRx(
                    portHandler,
                    static_cast<uint8_t>(motor_id),
                    ADDR_TORQUE_ENABLE,
                    1,
                    &dxl_error
                );
                if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to enable torque for motor " << motor_id);
                } else {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Torque enabled for motor " << motor_id);
                }
                rclcpp::sleep_for(std::chrono::milliseconds(50)); // Small delay between motor commands
            }
            current_state_ = RobotState::WAITING;
        }  

        // Initialize ROS 2 Services:
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stand_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sit_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr kneel_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr lie_down_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr give_paw_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr walk_service_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_torque_service_;
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
        int max_encoder_value_ = this->declare_parameter<int>("max_encoder_value", 4095);
        // Set the profile velocity and acceleration parameters:
        int profile_velocity_ = this->declare_parameter<int>("profile_velocity", 80);
        int profile_acceleration_ = this->declare_parameter<int>("profile_acceleration", 30);

        // Set the timer frequency:
        double rate_ = this->declare_parameter<double>("frequency", 20.0);
        // Initialize a timestep counter for the timer:
        uint64_t timestep_ = 0;

        // Initialize Motor Offset Map:
        std::unordered_map<int, int> motor_offset_map_;

        // Get the target joint angles for each pose:
        std::vector<double> standing_pose_ = this->declare_parameter<std::vector<double>>("standing_pose", std::vector<double>{});
        std::vector<double> sitting_pose_ = this->declare_parameter<std::vector<double>>("sitting_pose", std::vector<double>{});
        std::vector<double> kneeling_pose_ = this->declare_parameter<std::vector<double>>("kneeling_pose", std::vector<double>{});
        std::vector<double> lying_pose_ = this->declare_parameter<std::vector<double>>("lying_pose", std::vector<double>{});
        std::vector<double> paw_pose_ = this->declare_parameter<std::vector<double>>("paw_pose", std::vector<double>{});
        std::vector<double> walking_pose_sequence_ = this->declare_parameter<std::vector<double>>("walking_pose_sequence", std::vector<double>{});

        // Initializzation of Port Handlers:
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;

        // Initialize State Controller:
        RobotState current_state_ = RobotState::WAITING;

        // Track Current Motor Positions:
        std::vector<double> current_motor_positions_ = {};

        // Track Last Motor Ticks for Smooth Commanding:
        std::unordered_map<int, int32_t> last_motor_ticks_;

        // Walking Sequence State:
        int walking_phase_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticPositionNode>());
  rclcpp::shutdown();
  return 0;
}
