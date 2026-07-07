// MotionServo.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Stateless bang-bang approach control (sqrt(2*a*d) speed profile with clamped
// acceleration) for both translation and orientation. Pure kinematics in, force/
// torque demand out - no frame or mass handling. Leaf module.
//
// The commanded speed is capped to the remaining distance/angle so a single tick
// never carries past the target. This keeps the profile stable at coarse tick rates
// (where the uncapped sqrt term would exceed the remaining error and oscillate); at
// fine rates the cap is inert.
namespace MotionServo {

// Target approach velocity toward a point offset by `displacement`: sqrt(2*a*d)
// profile that comes to rest at the target when decelerating at `acceleration`.
// The caller owns the rest of the loop: pre-scale `acceleration` by its margin and
// near-field ramp (ramping down within a distance of the target prevents overshoot
// chatter at rest), then turn the returned velocity into a clamped acceleration -
// mirroring how towardDirection/towardOrientation pair with torque().
glm::dvec3 velocityToward(const glm::dvec3& displacement, double acceleration);

// 2-DOF servo: rotate `currentForward` onto `targetForward` around their common
// normal. Returns the target angular velocity; its magnitude scales with the
// remaining angle for precision near the target.
glm::dvec3 towardDirection(const glm::dvec3& currentForward,
                           const glm::dvec3& targetForward,
                           double maxAngularAcceleration, double margin);

// 3-DOF servo: rotate `current` onto `target`. Returns the target angular velocity;
// its magnitude ramps down linearly below `rampAngle` radians of remaining rotation
// so the approach settles without chatter.
glm::dvec3 towardOrientation(const glm::dquat& current, const glm::dquat& target,
                             double maxAngularAcceleration, double margin,
                             double rampAngle);

// Torque driving angular velocity toward targetAngularVelocity + extraAngularVelocity.
// The change is clamped to maxAngularAcceleration - the full, constant braking
// authority, kept separate from any near-target ramp on the target speed so residual
// velocity can still be arrested at the target.
glm::dvec3 torque(const glm::dvec3& targetAngularVelocity,
                  const glm::dvec3& extraAngularVelocity,
                  const glm::dvec3& currentAngularVelocity, double maxAngularAcceleration,
                  const glm::dmat3& inertia);

}
