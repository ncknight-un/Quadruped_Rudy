#include "rudylib/RudyKinematics.hpp"

namespace rudylib {
    RudyKinematics::RudyKinematics(
        double coxa,
        double femur,
        double tibia,
        std::vector<double> Abad_joint_limits,
        std::vector<double> Hip_joint_limits,
        std::vector<double> Knee_joint_limits,
        double x_leg_shift,
        double y_leg_shift
    ) {
        // Initialize the leg parameters:
        coxa_length_ = coxa;
        femur_length_ = femur;
        tibia_length_ = tibia;
        x_leg_shift_ = x_leg_shift;
        y_leg_shift_ = y_leg_shift;
        Abad_joint_limits_ = Abad_joint_limits;
        Hip_joint_limits_ = Hip_joint_limits;
        Knee_joint_limits_ = Knee_joint_limits;

        // Initialize leg base transforms to identity. (No Rotation, just translation from body to leg base)
        for (int i = 0; i < 4; i++)
            leg_base_transforms[i].setIdentity();
        // Establish the leg transforms based on their position to the robot base.
        leg_base_transforms[0].translation() = Eigen::Vector3d( -x_leg_shift,  y_leg_shift, 0.0); // FL
        leg_base_transforms[1].translation() = Eigen::Vector3d( -x_leg_shift, -y_leg_shift, 0.0); // BL
        leg_base_transforms[2].translation() = Eigen::Vector3d( x_leg_shift, -y_leg_shift, 0.0); // BR
        leg_base_transforms[3].translation() = Eigen::Vector3d( x_leg_shift,  y_leg_shift, 0.0); // FR

        // Initialize the current joint angles to zero:
        current_joint_angles_ = std::vector<JointAngles>(4, {0.0, 0.0, 0.0});
    } 

    // Return functions for foot positions relative to the body frame:
     Eigen::Vector3d RudyKinematics::get_pos_FL() const {
        Eigen::Vector3d foot_pos = front_left_leg.update_FK(current_joint_angles_[0]);
        return leg_base_transforms[0] * foot_pos;
    }
    Eigen::Vector3d RudyKinematics::get_pos_BL() const {
        Eigen::Vector3d foot_pos = back_left_leg.update_FK(current_joint_angles_[1]);
        return leg_base_transforms[1] * foot_pos;
    }
    Eigen::Vector3d RudyKinematics::get_pos_BR() const {
        Eigen::Vector3d foot_pos = back_right_leg.update_FK(current_joint_angles_[2]);
        return leg_base_transforms[2] * foot_pos;
    }
    Eigen::Vector3d RudyKinematics::get_pos_FR() const {
        Eigen::Vector3d foot_pos = front_right_leg.update_FK(current_joint_angles_[3]);
        return leg_base_transforms[3] * foot_pos;
    }

    // Given: body pose + joint angles → compute foot positions for all legs
    rudylib::foot_pos RudyKinematics::update_FK(const std::vector<JointAngles>& angles) {
        rudylib::foot_pos foot_positions;
        foot_positions.p_fl = front_left_leg.update_FK(angles[0]);
        foot_positions.p_fr = front_right_leg.update_FK(angles[1]);
        foot_positions.p_bl = back_left_leg.update_FK(angles[2]);
        foot_positions.p_br = back_right_leg.update_FK(angles[3]);

        // Update the current joint angles:
        for (int i = 0; i < 4; i++) {
            current_joint_angles_[i] = angles[i];
        }

        return foot_positions;
    }
    
    // Given: body pose + foot targets → compute joint angles for all legs
    std::vector<JointAngles> RudyKinematics::calc_IK(const rudylib::foot_pos& targets) {
        std::vector<JointAngles> joint_angles(4);
        joint_angles[0] = front_left_leg.calc_IK(targets.p_fl);
        joint_angles[1] = front_right_leg.calc_IK(targets.p_fr);
        joint_angles[2] = back_left_leg.calc_IK(targets.p_bl);
        joint_angles[3] = back_right_leg.calc_IK(targets.p_br);
        return joint_angles;
    }
}; // End of namespace rudylib