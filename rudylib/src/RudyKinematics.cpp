#include "RudyKinematics.hpp"

namespace rudylib {
    RudyKinematics::RudyKinematics(
        double coxa,
        double femur,
        double tibia,
        std::vector<double> Abad_joint_limits,
        std::vector<double> Hip_joint_limits,
        std::vector<double> Knee_joint_limits
    ) : 
        front_left_leg(coxa, femur, tibia, 'L', Abad_joint_limits, Hip_joint_limits, Knee_joint_limits),
        front_right_leg(coxa, femur, tibia, 'R', Abad_joint_limits, Hip_joint_limits, Knee_joint_limits),
        rear_left_leg(coxa, femur, tibia, 'L', Abad_joint_limits, Hip_joint_limits, Knee_joint_limits),
        rear_right_leg(coxa, femur, tibia, 'R', Abad_joint_limits, Hip_joint_limits, Knee_joint_limits)
    {
        // Initialize body and leg base transforms to identity (no transformation)
        body_transform.setIdentity();
        // Establoish the leg transforms based on their position to the robot base.
        leg_base_transforms[0].translation() = Eigen::Vector3d( x_front,  y_left, 0.0); // FL
        leg_base_transforms[1].translation() = Eigen::Vector3d( x_front, -y_right, 0.0); // FR
        leg_base_transforms[2].translation() = Eigen::Vector3d( x_rear,   y_left, 0.0); // RL
        leg_base_transforms[3].translation() = Eigen::Vector3d( x_rear,  -y_right, 0.0); // RR
    }

    // Given: body pose + joint angles → compute foot positions for all legs
    std::vector<Point3D> RudyKinematics::update_FK(std::vector<JointAngles> angles) {
        std::vector<Point3D> foot_positions(4);
        foot_positions[0] = front_left_leg.update_FK(angles[0]);
        foot_positions[1] = front_right_leg.update_FK(angles[1]);
        foot_positions[2] = rear_left_leg.update_FK(angles[2]);
        foot_positions[3] = rear_right_leg.update_FK(angles[3]);
        return foot_positions;
    }
    
    // Given: body pose + foot targets → compute joint angles for all legs
    std::vector<JointAngles> RudyKinematics::calc_IK(std::vector<Point3D> targets) {
        std::vector<JointAngles> joint_angles(4);
        joint_angles[0] = front_left_leg.calc_IK(targets[0]);
        joint_angles[1] = front_right_leg.calc_IK(targets[1]);
        joint_angles[2] = rear_left_leg.calc_IK(targets[2]);
        joint_angles[3] = rear_right_leg.calc_IK(targets[3]);
        return joint_angles;
    }
}; // End of namespace rudylib