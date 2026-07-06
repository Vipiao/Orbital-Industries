// DigibotFlyingMode.h
#pragma once

#include "DigibotModeTypes.h"
#include "../../physics/PhysicsUnits.h"
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
    double m_thrustStrength{PhysicsUnits::metersPerSecondSquared(16.384)};
    double m_angularAccelerationMax{PhysicsUnits::radiansPerSecondSquared(65.536)};
    double m_maxRollRate{PhysicsUnits::radiansPerSecond(2.048)};
    double m_translationLockStrength{PhysicsUnits::perSecond(64.0)};
};
