// MotionServo.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Stateless bang-bang approach control (sqrt(2*a*d) speed profile with clamped
// acceleration) for both translation and orientation. Pure kinematics in, force/
// torque demand out - no frame or mass handling. Leaf module.
namespace MotionServo {

// Target approach velocity toward a point offset by `displacement`: sqrt(2*a*d)
// profile that comes to rest at the target when decelerating at `acceleration`.
// The caller owns the rest of the loop: pre-scale `acceleration` by its margin and
// near-field ramp (ramping down within a distance of the target prevents overshoot
// chatter at rest), then turn the returned velocity into a clamped acceleration -
// mirroring how towardDirection/towardOrientation pair with torque().
glm::dvec3 velocityToward(const glm::dvec3& displacement, double acceleration);

struct AngularTarget {
    glm::dvec3 m_targetAngularVelocity{0.0, 0.0, 0.0};
    double m_accelerationLimit{0.0};
};

// 2-DOF servo: rotate `currentForward` onto `targetForward` around their common
// normal. Speed limit scales with the remaining angle (precise near the target).
AngularTarget towardDirection(const glm::dvec3& currentForward,
                              const glm::dvec3& targetForward,
                              double maxAngularAcceleration, double margin);

// 3-DOF servo: rotate `current` onto `target`. Acceleration ramps down linearly
// below `rampAngle` radians of remaining rotation.
AngularTarget towardOrientation(const glm::dquat& current, const glm::dquat& target,
                                double maxAngularAcceleration, double margin,
                                double rampAngle);

// Torque driving angular velocity toward target.m_targetAngularVelocity +
// extraAngularVelocity, with the acceleration clamped to target.m_accelerationLimit.
glm::dvec3 torque(const AngularTarget& target, const glm::dvec3& extraAngularVelocity,
                  const glm::dvec3& currentAngularVelocity, const glm::dmat3& inertia);

}
