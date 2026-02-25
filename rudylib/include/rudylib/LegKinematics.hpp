#ifndef RUDYLIB_KINEMATICS_INCLUDE_GUARD_HPP
#define RUDYLIB_KINEMATICS_INCLUDE_GUARD_HPP
/// \file
/// \brief Models the kinematics of a single Leg in the quadruped robot, Rudy.

#include <iosfwd>
#include <iostream>
#include <cmath>
#include <vector>
#include "rudylib/angle.hpp"
#include "Eigen/Dense"

namespace rudylib {
    /// \brief A struct to hold the joint angles of a single leg in the quadruped robot, Rudy.
    //
    // Abduction/Adduction, Hip, Knee in radians
    struct JointAngles {
        double abad = 0.0;
        double hip  = 0.0;
        double knee = 0.0;
    };

    /// \brief A class to model the kinematics of a single leg in the quadruped robot, Rudy.
    class QuadrupedLeg {
        public:
            // Constructor for the QuadrupedLeg class
            QuadrupedLeg(
                double coxa,
                double femur,
                double tibia,
                char legtype,
                std::vector<double> Abad_joint_limits,
                std::vector<double> Hip_joint_limits,
                std::vector<double> Knee_joint_limits
            );

            /// \brief Check if a target foot position is within the leg's domain
            /// \param target (Point3D) The desired foot position in 3D space
            /// \param coxa, femur, tibia (double) The lengths of the leg segments
            /// \param legtype (char) The type of leg ('L' or 'R
            /// \return bool True if the target is within the leg's domain, False otherwise
            bool is_within_domain(const Eigen::Vector3d& target);
            
            /// \brief Update the foot position given joint angles
            /// \param angles (JointAngles) The joint angles of the leg
            /// \return Point3D The updated foot position in 3D space
            Eigen::Vector3d update_FK(JointAngles angles) const;

            /// \brief Calculate the joint angles required to reach a target foot position
            /// \param target (Point3D) The desired foot position in 3D space
            /// \return JointAngles The joint angles required to reach the target position
            JointAngles calc_IK(Eigen::Vector3d target) const;

        private:
            double coxa_, femur_, tibia_ = 0.0; // Link lengths in meters
            char legtype = 'L'; // "L" or "R"
            std::vector<double> Abad_joint_limits = {0.0, 0.0}; // Joint limits in radians
            std::vector<double> Hip_joint_limits = {0.0, 0.0}; // Joint limits in radians
            std::vector<double> Knee_joint_limits = {0.0, 0.0}; // Joint limits in radians

    }; // End of class QuadrupedLeg
} // namespace rudylib
#endif // RUDYLIB_KINEMATICS_INCLUDE_GUARD_HPP