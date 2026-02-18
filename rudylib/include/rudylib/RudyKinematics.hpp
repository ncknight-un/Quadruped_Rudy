#ifndef RUDYLIB_RUDYKINEMATICS_HPP
#define RUDYLIB_RUDYKINEMATICS_HPP
/// \file 
/// \brief Models the kinematics of the quadruped robot for the whole body, Rudy, by utilizing the LegKinematics class for each leg.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rudylib/LegKinematics.hpp"

namespace rudylib
{
    /// \brief A class to model the kinematics of the whole body of the quadruped robot, Rudy, by utilizing the LegKinematics class for each leg.
    class RudyKinematics {
        public:
            // Constructor for the RudyKinematics class
            RudyKinematics(
                double coxa,
                double femur,
                double tibia,
                std::vector<double> Abad_joint_limits,
                std::vector<double> Hip_joint_limits,
                std::vector<double> Knee_joint_limits
            );

            /// \brief Update the foot positions of all legs given joint angles for each leg
            /// \param angles (std::vector<JointAngles>) The joint angles for each leg
            /// \return std::vector<Point3D> The updated foot positions for each leg in 3D space
            std::vector<Point3D> update_FK(std::vector<JointAngles> angles);

            /// \brief Calculate the joint angles required to reach target foot positions for all legs
            /// \param targets (std::vector<Point3D>) The desired foot positions for each leg in 3D space
            /// \return std::vector<JointAngles> The joint angles required to reach the target positions for each leg
            std::vector<JointAngles> calc_IK(std::vector<Point3D> targets);

        private:
            QuadrupedLeg front_left_leg;
            QuadrupedLeg front_right_leg;
            QuadrupedLeg rear_left_leg;
            QuadrupedLeg rear_right_leg;

            Eigen::Affine3d body_transform; // Transformation from body frame to world frame
            Eigen::Affine3d leg_base_transforms[4]; // Transformations from body frame to each leg's base frame
    };
};