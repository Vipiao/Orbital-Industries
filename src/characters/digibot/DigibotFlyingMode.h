// DigibotFlyingMode.h
#pragma once

#include "DigibotModeTypes.h"
#include <glm/glm.hpp>
#include <memory>

class RigidBody;

// Jetpack flight: view-relative thrust, optional velocity/rotation lock to a target
// grid body, and view-direction orientation control. Pure wrench producer — the
// controller applies the result.
class DigibotFlyingMode {
public:
    DigibotFlyingMode() = default;

    DigibotWrench update(const std::shared_ptr<RigidBody>& rigidBody,
                         const DigibotModeInputs& inputs,
                         DigibotLockState lockState,
                         const std::shared_ptr<RigidBody>& lockTarget);

    void setThrustStrength(double strength) { m_thrustStrength = strength; }
    void setMaxRollRate(double maxRate) { m_maxRollRate = maxRate; }

private:
    double m_thrustStrength{0.004};
    double m_angularAccelerationMax{0.016};  // Maximum angular acceleration (rad/s^2)
    double m_maxRollRate{0.032};             // Target roll rate (rad/s)
    double m_translationLockStrength{1.0};
};
