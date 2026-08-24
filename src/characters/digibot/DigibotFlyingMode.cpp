// DigibotFlyingMode.cpp
#include "DigibotFlyingMode.h"
#include "../ArticulationUtils.h"
#include "../MotionServo.h"
#include "../../physics/RigidBody.h"
#include "../../physics/RotatingFrameUtils.h"

DigibotWrench DigibotFlyingMode::update(const std::shared_ptr<RigidBody>& rigidBody,
                                        const DigibotModeInputs& inputs,
                                        DigibotLockState lockState,
                                        const std::shared_ptr<RigidBody>& lockTarget) {
    DigibotWrench wrench{};
    if (!rigidBody || rigidBody->getMass() <= 0.0) {
        return wrench;
    }

    // Lock force scaling parameter
    double lockForceScale{0.4};

    glm::dvec3 bodyUpDirection{rigidBody->getOrientation() * glm::dvec3{0.0, 0.0, 1.0}};

    // Note: conjugate because lookAt gives the inverse of the orientation from vectors.
    glm::dquat viewOrientation{glm::conjugate(
        ArticulationUtils::quatLookAtYForward(inputs.m_viewDirection, bodyUpDirection))};

    glm::dvec3 movementForce{0.0, 0.0, 0.0};
    if (inputs.m_movementDirection != glm::ivec3{0, 0, 0}) {
        glm::dvec3 direction{static_cast<double>(inputs.m_movementDirection.x),
                             static_cast<double>(inputs.m_movementDirection.y),
                             static_cast<double>(inputs.m_movementDirection.z)};
        if (glm::length(direction) > 0.0) {
            direction = glm::normalize(direction);
            // Transform direction from local to world space using view orientation
            direction = viewOrientation * direction;
            double forceMagnitude{m_thrustStrength * rigidBody->getMass()};
            movementForce = direction * forceMagnitude;
        }
    }

    glm::dvec3 lockForce{0.0, 0.0, 0.0};
    if (lockState == DigibotLockState::TRANSLATION_LOCK && lockTarget) {
        glm::dvec3 relativeVelocity{rigidBody->m_velocity - lockTarget->m_velocity};
        glm::dvec3 correctionForce{-relativeVelocity * m_translationLockStrength *
                                   rigidBody->getMass()};

        // Project correction force to plane orthogonal to movement force
        if (glm::length(movementForce) > 1e-6) {
            glm::dvec3 movementDirection{glm::normalize(movementForce)};
            double projection{glm::dot(correctionForce, movementDirection)};
            correctionForce = correctionForce - projection * movementDirection;
        }

        lockForce = correctionForce;

        double lockForceMagnitude{glm::length(lockForce)};
        double lockForceLimit{lockForceScale * m_thrustStrength * rigidBody->getMass()};
        if (lockForceMagnitude > lockForceLimit) {
            lockForce = lockForce * (lockForceLimit / lockForceMagnitude);
        }
    } else if (lockState == DigibotLockState::FULL_LOCK && lockTarget) {
        // Match the velocity of the material point of the grid at our position.
        glm::dvec3 targetLinearVelocity{
            RotatingFrameUtils::velocityAtPoint(*lockTarget, rigidBody->getPosition())};
        glm::dvec3 relativeLinearVelocity{rigidBody->m_velocity - targetLinearVelocity};

        glm::dvec3 correctionForce{-relativeLinearVelocity * m_translationLockStrength *
                                   rigidBody->getMass()};

        // Project correction force to plane orthogonal to movement force
        if (glm::length(movementForce) > 1e-6) {
            glm::dvec3 movementDirection{glm::normalize(movementForce)};
            double projection{glm::dot(correctionForce, movementDirection)};
            correctionForce = correctionForce - projection * movementDirection;
        }

        lockForce = correctionForce;

        double lockForceMagnitude{glm::length(lockForce)};
        double lockForceLimit{lockForceScale * m_thrustStrength * rigidBody->getMass()};
        if (lockForceMagnitude > lockForceLimit) {
            lockForce = lockForce * (lockForceLimit / lockForceMagnitude);
        }

        // Compensate centrifugal and coriolis forces of the co-rotating frame.
        glm::dvec3 radiusVector{rigidBody->getPosition() - lockTarget->getWorldCenterOfMass()};
        lockForce += RotatingFrameUtils::centrifugalCoriolisCompensation(
            rigidBody->getMass(), lockTarget->getAngularVelocityWorld(), radiusVector,
            relativeLinearVelocity);
    }

    // Movement and lock share one thrust budget, so a locked bot cannot also
    // accelerate at full strength.
    double maxForce{m_thrustStrength * rigidBody->getMass()};
    glm::dvec3 totalForce{movementForce + lockForce};
    double totalForceMagnitude{glm::length(totalForce)};
    if (totalForceMagnitude > maxForce) {
        totalForce = totalForce * (maxForce / totalForceMagnitude);
    }
    if (totalForceMagnitude > 1e-6) {
        wrench.m_force = totalForce;
    }

    glm::dvec3 currentForward{
        glm::normalize(rigidBody->getOrientation() * glm::dvec3{0.0, 1.0, 0.0})};
    glm::dvec3 targetForward{glm::normalize(inputs.m_viewDirection)};

    glm::dvec3 targetAngularVelocity{MotionServo::towardDirection(
        currentForward, targetForward, m_angularAccelerationMax, 0.2)};

    // Roll around the view direction plus co-rotation with a fully locked grid.
    glm::dvec3 extraAngularVelocity{0.0, 0.0, 0.0};
    glm::dvec3 rollAxis{glm::normalize(inputs.m_viewDirection)};
    extraAngularVelocity += static_cast<double>(inputs.m_rollInput) * m_maxRollRate *
                            rollAxis;
    if (lockState == DigibotLockState::FULL_LOCK && lockTarget) {
        extraAngularVelocity += lockTarget->getAngularVelocityWorld();
    }

    wrench.m_torque = MotionServo::torque(
        targetAngularVelocity, extraAngularVelocity, rigidBody->getAngularVelocityWorld(),
        m_angularAccelerationMax, rigidBody->getWorldInertiaTensor());

    return wrench;
}
