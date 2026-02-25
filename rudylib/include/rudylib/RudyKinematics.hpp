#ifndef RUDYLIB_RUDYKINEMATICS_HPP
#define RUDYLIB_RUDYKINEMATICS_HPP
/// \file 
/// \brief Models the kinematics of the quadruped robot for the whole body, Rudy, by utilizing the LegKinematics class for each leg.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rudylib/LegKinematics.hpp"

namespace rudylib {
    /// \brief a custom struct to hold the position of each of the feet relative to the body
    struct foot_pos{
        /// \brief the left front foot
        Eigen::Vector3d p_fl = Eigen::Vector3d(0.0, 0.0, 0.0);
        /// \brief the left back foot
        Eigen::Vector3d p_bl = Eigen::Vector3d(0.0, 0.0, 0.0);
        /// \brief the right back foot
        Eigen::Vector3d p_br = Eigen::Vector3d(0.0, 0.0, 0.0);
        /// \brief the right front foot
        Eigen::Vector3d p_fr = Eigen::Vector3d(0.0, 0.0, 0.0);
    };

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
                std::vector<double> Knee_joint_limits,
                double x_leg_shift,
                double y_leg_shift
            );

            /// \brief get the position of the front left foot:
            /// \return the 3D position of the foot relative to the body.
            Eigen::Vector3d get_pos_FL() const;

            /// \brief get the position of the back left foot:
            /// \return the 3D position of the foot relative to the body.
            Eigen::Vector3d get_pos_BL() const;

            /// \brief get the position of the back right foot:
            /// \return the 3D position of the foot relative to the body.
            Eigen::Vector3d get_pos_BR() const;

            /// \brief get the position of the front right foot:
            /// \return the 3D position of the foot relative to the body.
            Eigen::Vector3d get_pos_FR() const;

            /// \brief Update the foot positions of all legs given joint angles for each leg
            /// \param angles (std::vector<JointAngles>) The joint angles for each leg
            /// \return rudylib::foot_pos The updated foot positions for each leg in 3D space
            rudylib::foot_pos update_FK(const std::vector<JointAngles>& angles);

            /// \brief Calculate the joint angles required to reach target foot positions for all legs
            /// \param targets (rudylib::foot_pos) The desired foot positions for each leg in 3D space
            /// \return std::vector<JointAngles> The joint angles required to reach the target positions for each leg
            std::vector<JointAngles> calc_IK(const rudylib::foot_pos& targets);

        private:
            // Save each legs leg component lengths:
            double coxa_length_;
            double femur_length_;
            double tibia_length_;

            // The translation component of each hip origin to the base frame:
            double x_leg_shift_;
            double y_leg_shift_;

            // The joint limits for each joint type:
            std::vector<double> Abad_joint_limits_;
            std::vector<double> Hip_joint_limits_;
            std::vector<double> Knee_joint_limits_;

            // Create a leg kinematics object for each leg of the robot:
            QuadrupedLeg front_left_leg = rudylib::QuadrupedLeg(coxa_length_, femur_length_, tibia_length_, 'L', Abad_joint_limits_, Hip_joint_limits_, Knee_joint_limits_);
            QuadrupedLeg front_right_leg = rudylib::QuadrupedLeg(coxa_length_, femur_length_, tibia_length_, 'R', Abad_joint_limits_, Hip_joint_limits_, Knee_joint_limits_);
            QuadrupedLeg back_left_leg = rudylib::QuadrupedLeg(coxa_length_, femur_length_, tibia_length_, 'L', Abad_joint_limits_, Hip_joint_limits_, Knee_joint_limits_);
            QuadrupedLeg back_right_leg = rudylib::QuadrupedLeg(coxa_length_, femur_length_, tibia_length_, 'R', Abad_joint_limits_, Hip_joint_limits_, Knee_joint_limits_);   

            // Store the current leg transfroms to body transform:
            Eigen::Affine3d leg_base_transforms[4];

            // Hold the current joint angles of each leg:
            std::vector<JointAngles> current_joint_angles_;
    };
};
#endif // RUDYLIB_RUDYKINEMATICS_HPP