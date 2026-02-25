#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "rudylib/LegKinematics.hpp"
#include "rudylib/RudyKinematics.hpp"
#include <numbers>
#include <cmath>
#include <cstdlib>
#include <vector>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

// Create a function to make a test body kinematics model for Catch2 tests:
rudylib::RudyKinematics makeTestBody() {
    return rudylib::RudyKinematics(
        0.023,   // coxa
        0.077,   // femur
        0.1,   // tibia
        {-0.25, 1.25},    // Abad joint limits
        {-1.5, 1.5},    // Hip joint limits
        {-1.9, 1.9},    // Knee joint limits
        0.08,   // x leg shift from body center
        0.07    // y leg shift from body center
    );
}

TEST_CASE("FK-FK consistency - all angles", "[fk]") {
    auto body = makeTestBody();

    std::vector<rudylib::JointAngles> input = {
        {0.2, -0.4, -0.8},   // FL
        {0.1, 0.3, -0.5},    // FR
        {-0.2, 0.5, -1.0},   // BL
        {-0.1, -0.3, -0.7}   // BR
    };

    rudylib::foot_pos target = body.update_FK(input);
    rudylib::foot_pos reconstructed = body.update_FK(input);

    REQUIRE_THAT(reconstructed.p_fl.x(), WithinRel(target.p_fl.x(), EPS));
    REQUIRE_THAT(reconstructed.p_fl.y(), WithinRel(target.p_fl.y(), EPS));
    REQUIRE_THAT(reconstructed.p_fl.z(), WithinRel(target.p_fl.z(), EPS));

    REQUIRE_THAT(reconstructed.p_fr.x(), WithinRel(target.p_fr.x(), EPS));
    REQUIRE_THAT(reconstructed.p_fr.y(), WithinRel(target.p_fr.y(), EPS));
    REQUIRE_THAT(reconstructed.p_fr.z(), WithinRel(target.p_fr.z(), EPS));

    REQUIRE_THAT(reconstructed.p_bl.x(), WithinRel(target.p_bl.x(), EPS));
    REQUIRE_THAT(reconstructed.p_bl.y(), WithinRel(target.p_bl.y(), EPS));
    REQUIRE_THAT(reconstructed.p_bl.z(), WithinRel(target.p_bl.z(), EPS));

    REQUIRE_THAT(reconstructed.p_br.x(), WithinRel(target.p_br.x(), EPS));
    REQUIRE_THAT(reconstructed.p_br.y(), WithinRel(target.p_br.y(), EPS));
    REQUIRE_THAT(reconstructed.p_br.z(), WithinRel(target.p_br.z(), EPS));
}