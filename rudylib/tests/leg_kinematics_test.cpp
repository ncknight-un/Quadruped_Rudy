#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "rudylib/LegKinematics.hpp"
#include <numbers>
#include <cmath>
#include <cstdlib>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

// Create a function to make a test leg for Catch2 tests:
rudylib::QuadrupedLeg makeTestLeg(char type = 'L') {
    return rudylib::QuadrupedLeg(
        0.05,   // coxa
        0.10,   // femur
        0.15,   // tibia
        type,   // 'L' or 'R'
        {-1.0, 1.0},
        {-1.5, 1.5},
        {-2.0, 0.0}
    );
}

TEST_CASE("Target inside global reach is valid", "[domain]") {
    auto leg = makeTestLeg();
    Eigen::Vector3d target{0.05, 0.10, 0.10};

    REQUIRE(leg.is_within_domain(target));
}

TEST_CASE("Target outside global reach is invalid", "[domain]") {
    auto leg = makeTestLeg();

    // Max reach = 0.05 + 0.10 + 0.15 = 0.30
    Eigen::Vector3d target{0.5, 0.0, 0.0};

    REQUIRE_FALSE(leg.is_within_domain(target));
}

TEST_CASE("Target violating triangle inequality is invalid", "[domain]") {
    auto leg = makeTestLeg();

    // Extremely close to hip
    Eigen::Vector3d target{0.05, 0.0, 0.05};

    REQUIRE_FALSE(leg.is_within_domain(target));
}

TEST_CASE("FK with zero angles gives straight vertical leg", "[fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles a{0.0, 0.0, 0.0};
    Eigen::Vector3d p = leg.update_FK(a);

    REQUIRE_THAT(p.x(), WithinRel(0.0, EPS));
    REQUIRE_THAT(p.y(), WithinRel(0.0, EPS));
    REQUIRE_THAT(p.z(), WithinRel(-(0.05 + 0.10 + 0.15), EPS));
}

TEST_CASE("FK coxia flexion moves foot forward in X", "[fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles a{std::numbers::pi/2.0, 0.0, 0.0};
    Eigen::Vector3d p = leg.update_FK(a);

    REQUIRE_THAT(p.x(), WithinRel(0.05 + 0.10 + 0.15, EPS));  // Should be 0.3, as the leg is fully extended in the X direction
    REQUIRE_THAT(p.y(), WithinRel(0.0, EPS));
}

TEST_CASE("FK hip flexion moves foot forward in Y", "[fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles a{0.0, std::numbers::pi/2.0, 0.0};
    Eigen::Vector3d p = leg.update_FK(a);

    REQUIRE_THAT(p.x(), WithinRel(0.0, EPS));
    REQUIRE_THAT(p.y(), WithinRel(0.10 + 0.15, EPS));
    REQUIRE_THAT(p.z(), WithinRel(-0.05, EPS));
}

TEST_CASE("FK knee flexion moves foot forward in Y", "[fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles a{0.0, 0.0, std::numbers::pi/2.0};
    Eigen::Vector3d p = leg.update_FK(a);

    REQUIRE_THAT(p.x(), WithinRel(0.0, EPS));
    REQUIRE_THAT(p.y(), WithinRel(0.15, EPS));
    REQUIRE_THAT(p.z(), WithinRel(-(0.05 + 0.10), EPS));
}

TEST_CASE("FK for all joints", "[fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles a{std::numbers::pi/4.0, std::numbers::pi/4.0, std::numbers::pi/4.0};
    Eigen::Vector3d p = leg.update_FK(a);

    REQUIRE_THAT(p.x(), WithinRel(0.085355, EPS));
    REQUIRE_THAT(p.y(), WithinRel(0.220711, EPS));
    REQUIRE_THAT(p.z(), WithinRel(-0.085355, EPS));
}

TEST_CASE("IK-FK roundtrip consistency - abad only", "[ik][fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles input{0.2, 0.0, 0.0};

     Eigen::Vector3d target = leg.update_FK(input);
    rudylib::JointAngles solved = leg.calc_IK(target);
    Eigen::Vector3d reconstructed = leg.update_FK(solved);

    REQUIRE_THAT(reconstructed.x(), WithinRel(target.x(), EPS));
    REQUIRE_THAT(reconstructed.y(), WithinRel(target.y(), EPS));
    REQUIRE_THAT(reconstructed.z(), WithinRel(target.z(), EPS));
}

TEST_CASE("IK-FK roundtrip consistency - hip only", "[ik][fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles input{0.0, 0.2, 0.0};

     Eigen::Vector3d target = leg.update_FK(input);
    rudylib::JointAngles solved = leg.calc_IK(target);
    Eigen::Vector3d reconstructed = leg.update_FK(solved);

    REQUIRE_THAT(reconstructed.x(), WithinRel(target.x(), EPS));
    REQUIRE_THAT(reconstructed.y(), WithinRel(target.y(), EPS));
    REQUIRE_THAT(reconstructed.z(), WithinRel(target.z(), EPS));
}

TEST_CASE("IK-FK roundtrip consistency - knee only", "[ik][fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles input{0.0, 0.0, 0.8};

    Eigen::Vector3d target = leg.update_FK(input);
    rudylib::JointAngles solved = leg.calc_IK(target);
    Eigen::Vector3d reconstructed = leg.update_FK(solved);

    REQUIRE_THAT(reconstructed.x(), WithinRel(target.x(), EPS));
    REQUIRE_THAT(reconstructed.y(), WithinRel(target.y(), EPS));
    REQUIRE_THAT(reconstructed.z(), WithinRel(target.z(), EPS));
}


TEST_CASE("IK-FK roundtrip consistency - all angles", "[ik][fk]") {
    auto leg = makeTestLeg();

    rudylib::JointAngles input{0.2, -0.4, -0.8};

    Eigen::Vector3d target = leg.update_FK(input);
    rudylib::JointAngles solved = leg.calc_IK(target);
    Eigen::Vector3d reconstructed = leg.update_FK(solved);
    REQUIRE_THAT(reconstructed.x(), WithinRel(target.x(), EPS));
    REQUIRE_THAT(reconstructed.y(), WithinRel(target.y(), EPS));
    REQUIRE_THAT(reconstructed.z(), WithinRel(target.z(), EPS));
}

TEST_CASE("Right leg flips angle signs", "[ik][symmetry]") {
    auto left = makeTestLeg('L');
    auto right = makeTestLeg('R');

    rudylib::JointAngles a{0.2, -0.4, -0.8};
    Eigen::Vector3d target = left.update_FK(a);

    rudylib::JointAngles right_solution = right.calc_IK(target);

    REQUIRE_THAT(right_solution.abad, WithinRel(-a.abad, EPS));
    REQUIRE_THAT(right_solution.hip,  WithinRel(a.hip, EPS));
    REQUIRE_THAT(right_solution.knee, WithinRel(-a.knee, EPS));
}
