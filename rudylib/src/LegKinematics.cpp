#include "rudylib/LegKinematics.hpp"

namespace rudylib {
    QuadrupedLeg::QuadrupedLeg(double coxa, double femur, double tibia, char legtype, std::vector<double> Abad_joint_limits, std::vector<double> Hip_joint_limits, std::vector<double> Knee_joint_limits)
    : coxa_(coxa), femur_(femur), tibia_(tibia), legtype(legtype), Abad_joint_limits(Abad_joint_limits), Hip_joint_limits(Hip_joint_limits), Knee_joint_limits(Knee_joint_limits) {} // End of constructor

    bool QuadrupedLeg::is_within_domain(const Eigen::Vector3d& target) {
        // Check the Global Domain (L):
        double L =  sqrt(target.x() * target.x() + target.y() * target.y() + target.z() * target.z());
        // Get the Legs Max Reach (M):
        double M = coxa_ + femur_ + tibia_;
        // If the target is outside the leg's reach, return false
        if (L > M) {
            return false;   // Target is outside the leg's reach
        }

        // Check the Lower Leg Domain (K):
        double phi_abad = atan2(target.x(), target.z());    // Abduction angle to reach the target
        double x_eff = coxa_ * sin(phi_abad);
        double z_eff = coxa_ * cos(phi_abad);
        double K = sqrt(pow(target.x() - x_eff, 2) + pow(target.y(), 2) + pow(target.z() - z_eff, 2));
        // If the target is outside the lower leg's reach, return false
        if (K > (femur_ + tibia_) || K < fabs(femur_ - tibia_)) {       // Check if K is greater than reach or triangle is invalid.
            return false;   // Target is outside the lower leg's reach  
        }

        // Both checks pass, the target is within the leg's domain:
        return true;
    }

    // Forward Kinematics: Angles to Position
    Eigen::Vector3d QuadrupedLeg::update_FK(JointAngles angles) const {
        // First find the position of the hip joint after abduction
        double x_hip = coxa_ * sin(angles.abad);
        double z_hip = coxa_ * cos(angles.abad);

        // Calculate the reach in the femur-tibia plane (I assume hip angle 0 is straight down leg)
        double r = femur_ * sin(angles.hip) + tibia_ * sin(angles.hip + angles.knee);       // Reach in the Y direction
        double h = femur_ * cos(angles.hip) + tibia_ * cos(angles.hip + angles.knee);       // Height in the Z direction
        
        // Calculate the foot position in 3D space:
        Eigen::Vector3d foot;
        foot.x() = x_hip + h * sin(angles.abad);  // Total abduction/adduction offset in the X direction
        foot.y() = r;  // Total forward/backward reach in the Y direction
        foot.z() = -(z_hip + h * cos(angles.abad)); // Total height in the Z direction
        
        // Return the x, y, z position of the foot given the joint angles:
        return foot;
    }

    // Inverse Kinematics: Position to Angles
    JointAngles QuadrupedLeg::calc_IK(Eigen::Vector3d target) const {
        JointAngles result;

        // Solve Abduction Angle:
        result.abad = atan2(target.x(), -target.z());

        // Reposition effective lower leg origin at hip origin moving Hip Joint:
        double x_eff = coxa_ * sin(result.abad);    // Shift in X due to abduction
        double z_eff = coxa_ * cos(result.abad);    // Shift in Z due to abduction

        // Solve 3D distance K - the effective distance from the hip joint to the target foot position:
        double dx = target.x() - x_eff;
        double dy = target.y();           // No shift in Y since abduction doesn't affect forward/backward position
        double dz = -target.z() - z_eff;
        double K = sqrt(dx*dx + dy*dy + dz*dz);

        // Solve Knee Angle (Law of Cosines):
        double cos_knee = (femur_*femur_ + tibia_*tibia_ - K*K) / (2 * femur_ * tibia_);
        result.knee = std::numbers::pi + acos(cos_knee);
        // Wrap the knee angle to the range [-pi, pi]:
        result.knee = rudylib::normalize_angle(result.knee);

        // Solve Hip Angle:
        double alpha1 = atan2(dy, sqrt(dx*dx + dz*dz)); // Angle from hip to target in the horizontal plane
        double cos_alpha2 = (femur_*femur_ + K*K - tibia_*tibia_) / (2 * femur_ * K);   // Law of Cosines for the angle between femur and the line from hip to target
        cos_alpha2 = std::max(-1.0, std::min(1.0, cos_alpha2));     // Constrain to avoid NaN if target is out of reach
        double alpha2 = acos(cos_alpha2);
        result.hip = alpha1 + alpha2;

        // Flip the angles for the right legs to maintain consistent positive direction convention:
        // Note: If you reproduce this model, you may need to adjust this if you orient your legs differently.
        if (legtype == 'R') {
            result.abad = -result.abad;   // Flip abduction angle for right legs
            result.hip = result.hip;     // Hips are oriented the same for left and right legs, so no flip needed
            result.knee = -result.knee;   // Flip knee angle for right legs
        }
        return result;
    }
};