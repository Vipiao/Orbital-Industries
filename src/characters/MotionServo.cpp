// MotionServo.cpp
#include "MotionServo.h"

namespace MotionServo {

glm::dvec3 velocityToward(const glm::dvec3& displacement, double acceleration) {
    double distance{glm::length(displacement)};
    if (distance < 1e-9 || acceleration <= 0.0) {
        return glm::dvec3{0.0, 0.0, 0.0};
    }
    // Cap the commanded speed to the remaining distance: over one tick the body moves
    // `speed` (tick units), so speed <= distance stops the sqrt(2ad) profile from
    // carrying past the target and oscillating at coarse tick rates. At fine rates the
    // sqrt term is always the smaller one, so the cap is inert.
    double speed{glm::min(std::sqrt(2.0 * acceleration * distance), distance)};
    return (displacement / distance) * speed;
}

AngularTarget towardDirection(const glm::dvec3& currentForward,
                              const glm::dvec3& targetForward,
                              double maxAngularAcceleration, double margin) {
    AngularTarget result{};
    result.m_accelerationLimit = maxAngularAcceleration;

    double dotProduct{glm::clamp(glm::dot(currentForward, targetForward), -1.0, 1.0)};
    double deltaAngle{std::acos(dotProduct)};

    glm::dvec3 rotationAxis{glm::cross(currentForward, targetForward)};
    double axisLength{glm::length(rotationAxis)};

    // Speed limit scales with the remaining angle for precision near the target.
    double adjustedAngVelMax{maxAngularAcceleration * glm::abs(deltaAngle)};

    if (axisLength > 1e-6) {
        rotationAxis = rotationAxis / axisLength;
        // Cap to the remaining angle so one tick never rotates past the target.
        double maxAngularSpeed{glm::min(
            std::sqrt(2.0 * adjustedAngVelMax * (1.0 - margin) * deltaAngle), deltaAngle)};
        result.m_targetAngularVelocity = rotationAxis * maxAngularSpeed;
    }
    return result;
}

AngularTarget towardOrientation(const glm::dquat& current, const glm::dquat& target,
                                double maxAngularAcceleration, double margin,
                                double rampAngle) {
    AngularTarget result{};

    // Take the short path: q and -q represent the same rotation.
    glm::dquat shortTarget{target};
    if (glm::dot(shortTarget, current) < 0.0) {
        shortTarget = -shortTarget;
    }

    glm::dquat rotationDelta{shortTarget * glm::conjugate(current)};
    double deltaAngle{2.0 * std::acos(glm::clamp(rotationDelta.w, -1.0, 1.0))};
    glm::dvec3 rotationAxis{rotationDelta.x, rotationDelta.y, rotationDelta.z};
    double axisLength{glm::length(rotationAxis)};

    // Ramp acceleration down when close to the target orientation.
    double effectiveAcceleration{maxAngularAcceleration *
                                 glm::min(glm::abs(deltaAngle) / rampAngle, 1.0)};
    result.m_accelerationLimit = effectiveAcceleration;

    if (axisLength > 1e-6 && deltaAngle > 1e-6) {
        rotationAxis = rotationAxis / axisLength;
        // Cap to the remaining angle so one tick never rotates past the target.
        double maxAngularSpeed{glm::min(
            std::sqrt(2.0 * effectiveAcceleration * (1.0 - margin) * deltaAngle), deltaAngle)};
        result.m_targetAngularVelocity = rotationAxis * maxAngularSpeed;
    }
    return result;
}

glm::dvec3 torque(const AngularTarget& target, const glm::dvec3& extraAngularVelocity,
                  const glm::dvec3& currentAngularVelocity, const glm::dmat3& inertia) {
    glm::dvec3 targetAngularVelocity{target.m_targetAngularVelocity +
                                     extraAngularVelocity};
    glm::dvec3 angularAcceleration{targetAngularVelocity - currentAngularVelocity};

    double magnitude{glm::length(angularAcceleration)};
    if (magnitude > target.m_accelerationLimit) {
        angularAcceleration =
            angularAcceleration * (target.m_accelerationLimit / magnitude);
    }
    return inertia * angularAcceleration;
}

}
