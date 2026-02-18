#ifndef RUDYLIB_ANGLE_HPP_INCLUDE_GUARD
#define RUDYLIB_ANGLE_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles
#include <iostream>
#include <cmath>
#include <numbers>

namespace rudylib
{
        /// \brief Approximately compare two floating-point numbers using an absolute comparison
    /// \param d1 A number to compare
    /// \param d2 A second number to compare
    /// \param epsilon Absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
{
    // If the abs value of the distance is less than the epsilon value, then return true:
    return std::abs(d1 - d2) < epsilon;
}

    /// \brief Convert degrees to radians
    /// \param deg Angle in degrees
    /// \returns The equivalent angle in radians
constexpr double deg2rad(double deg)
{
        // Return the conversion of radians from degrees.
  using std::numbers::pi;
  return  deg * (pi / 180);
}

    /// \brief Convert radians to degrees
    /// \param rad  Angle in radians
    /// \returns The equivalent angle in degrees
constexpr double rad2deg(double rad)
{
        // Return the conversion of degrees from radians.
  using std::numbers::pi;
  return  rad * (180 / pi);
}

    /// \brief Wrap an angle to (-PI, PI]
    /// \param rad Angle in radians
    /// \return An equivalent angle the range (-PI, PI]
constexpr double normalize_angle(double rad)
{
        // Wrap an angle from -Pi to Pi:
  using std::numbers::pi;
  rad = std::fmod(rad, 2.0 * pi);         // wrap to [-2*pi, 2*pi] using float mod operator.

        // Shift into our desired range:
  if (rad <= -pi) {
    rad += 2.0 * pi;                        // shift into (-pi, pi] because we don't want to include -pi
  } else if (rad > pi) {
    rad -= 2.0 * pi;
  }
        // Return the wrapped result:
  return rad;
}

    /// static_assertions test compile time assumptions.
    /// These tests can provide assurance that your code is correct at compile time!
static_assert(almost_equal(0, 0), "is_zero failed");
    /// 1. Compare two numbers where almost_equal is true or false depending on the epsilon argument
    ///    almost_equal(x, y, e1) == true
    ///    almost_equal(x, y, e2) == false
static_assert(almost_equal(0.0, 0.01, 0.02), "is_equal failed");
static_assert(!almost_equal(0.0, 1.0, 0.1), "is_equal failed");             // Case where it fails (We use NOT operator to let compiler pass)
    /// 2. Compare negative and positive numbers that should not be equal
static_assert(!almost_equal(-0.001, 0.001, 0.001), "is_equal failed");
    /// 3. Compare negative and positive numbers that should be equal
static_assert(almost_equal(-0.001, 0.001, 0.0025), "is_equal failed");

static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    /// 4 additional tests for deg2rad. Include at least one negative angle
    /// Ex 1: 45 degrees and passes
static_assert(almost_equal(deg2rad(45.0), std::numbers::pi / 4), "deg2rad failed");
    /// Ex 2: -45 degrees and passes
static_assert(almost_equal(deg2rad(-45.0), -1 * std::numbers::pi / 4), "deg2rad failed");
    /// Ex 3: 90 degrees and passes
static_assert(almost_equal(deg2rad(90.0), std::numbers::pi / 2), "deg2rad failed");
    /// Ex 4: 180 degrees and fails
static_assert(almost_equal(deg2rad(180.0), std::numbers::pi), "deg2rad failed");

static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    /// 4 additional tests for rad2deg. Include at least one negative angle
    /// Ex 1: pi/4:
static_assert(almost_equal(rad2deg(std::numbers::pi / 4), 45.0), "rad2deg failed");
    /// Ex 2: -pi/4:
static_assert(almost_equal(rad2deg(-1 * std::numbers::pi / 4), -45.0), "rad2deg failed");
    /// Ex 3: pi/2:
static_assert(almost_equal(rad2deg(std::numbers::pi / 2), 90.0), "rad2deg failed");
    /// Ex 4: pi:
static_assert(almost_equal(rad2deg(std::numbers::pi), 180.0), "rad2deg failed");

static_assert(almost_equal(normalize_angle(0.0), 0.0), "norm_angle failed");
static_assert(almost_equal(normalize_angle(std::numbers::pi), std::numbers::pi),
    "norm_angle failed");
static_assert(almost_equal(normalize_angle(-std::numbers::pi), std::numbers::pi),
    "norm_angle failed");                                                                                       // We are wrapping to not include -pi
    /// Additional tests for normalize_angle. This function is absolutely critical, so you want to get it right!
    // Case where the angle > 5.0*pi
static_assert(almost_equal(normalize_angle(8.4 * std::numbers::pi), 0.4 * std::numbers::pi),
    "norm_angle failed");
    // Case where the angle < -5.0*pi
static_assert(almost_equal(normalize_angle(-12.4 * std::numbers::pi), -0.4 * std::numbers::pi),
    "norm_angle failed");
    // Case -pi/4.0:
static_assert(almost_equal(normalize_angle(-1 * std::numbers::pi / 4), -1 * std::numbers::pi / 4),
    "norm_angle failed");
    // Case 3*pi/2:
static_assert(almost_equal(normalize_angle(3 * std::numbers::pi / 2), -1 * std::numbers::pi / 2),
    "norm_angle failed");
    // Case -5*pi/2:
static_assert(almost_equal(normalize_angle(-5 * std::numbers::pi / 2), -1 * std::numbers::pi / 2),
    "norm_angle failed");
    // Case pi/4.0:
static_assert(almost_equal(normalize_angle(std::numbers::pi / 4), std::numbers::pi / 4),
    "norm_angle failed");
    // Case -3*pi/2:
static_assert(almost_equal(normalize_angle(-3 * std::numbers::pi / 2), std::numbers::pi / 2),
    "norm_angle failed");
    // Case 5*pi/2:
static_assert(almost_equal(normalize_angle(5 * std::numbers::pi / 2), std::numbers::pi / 2),
    "norm_angle failed");
    // Case -2*pi/5:
static_assert(almost_equal(normalize_angle(-2 * std::numbers::pi / 5), -2 * std::numbers::pi / 5),
    "norm_angle failed");
    // Case 2pi/5.0:
static_assert(almost_equal(normalize_angle(2 * std::numbers::pi / 5), 2 * std::numbers::pi / 5),
    "norm_angle failed");
    // Case -6*pi:
static_assert(almost_equal(normalize_angle(-6 * std::numbers::pi), 0.0), "norm_angle failed");
    // Case 5*pi:
static_assert(almost_equal(normalize_angle(5 * std::numbers::pi), std::numbers::pi),
    "norm_angle failed");
}
#endif
