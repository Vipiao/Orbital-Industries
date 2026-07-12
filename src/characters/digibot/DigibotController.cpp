// DigibotController.cpp
#include "DigibotController.h"
#include "DigibotPhysics.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/RigidBody.h"
#include <glm/gtx/vector_angle.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

DigibotController::DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine)
    : m_physics{physics}
    , m_physicsEngine{physicsEngine}
    , m_walkingMode{physics, physicsEngine}
{
    if (!m_physics) {
        throw std::runtime_error("DigibotController: Physics component cannot be null");
    }
    if (!m_physicsEngine) {
        throw std::runtime_error("DigibotController: Physics engine cannot be null");
    }
}

void DigibotController::setViewDirection(const glm::dvec3& viewDirection) {
    // Normalize to ensure it's a unit direction vector
    m_viewDirection = glm::normalize(viewDirection);
}

void DigibotController::setMovementDirection(const glm::ivec3& direction) {
    m_movementDirection = direction;
}

void DigibotController::setRollInput(int rollInput) {
    m_rollInput = rollInput;
}

void DigibotController::setJetpackEnabled(bool enabled) {
    m_isJetpackEnabled = enabled;
    if (enabled) {
        m_walkingMode.resetContactState();
    }
    std::cout << "Jetpack " << (m_isJetpackEnabled ? "ENABLED" : "DISABLED") << std::endl;
}

void DigibotController::setTargetRigidBody(std::weak_ptr<RigidBody> rigidBody) {
    m_targetRigidBody = rigidBody;
    std::cout << "Target rigid body set" << std::endl;
}

void DigibotController::unlock() {
    m_targetRigidBody.reset();
    m_lockState = DigibotLockState::UNLOCKED;
    std::cout << "Unlocked" << std::endl;
}

void DigibotController::setThrustStrength(double strength) {
    m_flyingMode.setThrustStrength(strength);
}

void DigibotController::setMaxRollRate(double maxRate) {
    m_flyingMode.setMaxRollRate(maxRate);
}

void DigibotController::setDockingTarget(const DigibotDockingMode::Target& target) {
    m_dockingTarget = target;
    if (m_dockingState == DockingState::FREE) {
        // Snapshot the free-movement settings so we return exactly as we left.
        // Walking's cached grid-space up survives untouched: walking simply does not
        // run while docked (no reset), so it is still valid on exit.
        m_savedLockState = m_lockState;
        m_savedTargetRigidBody = m_targetRigidBody;
        m_savedJetpackEnabled = m_isJetpackEnabled;

        m_dockingState = DockingState::DOCKED;
        m_dockingMode.armSeatCapture();
        for (Collider* cell : m_dockingTarget.m_cockpitCells) {
            m_physics->addBodyCollisionExceptionWith(cell);
        }
        std::cout << "Docking: entering cockpit corridor" << std::endl;
    }
}

void DigibotController::clearDockingTarget() {
    if (m_dockingState != DockingState::FREE) {
        releaseDocking();
    }
}

void DigibotController::requestUnseat() {
    if (m_dockingState == DockingState::SEATED) {
        m_dockingState = DockingState::DOCKED;
        m_dockingMode.disarmSeatCapture();
        std::cout << "Unseated: back in the docking corridor" << std::endl;
    }
}

void DigibotController::releaseDocking() {
    m_physics->clearBodyCollisionExceptions();
    m_dockingState = DockingState::FREE;
    m_dockingTarget = DigibotDockingMode::Target{};

    // Restore the free-movement settings captured on entry. Set members directly
    // (not via setJetpackEnabled) so walking's contact cache is not reset.
    m_lockState = m_savedLockState;
    m_targetRigidBody = m_savedTargetRigidBody;
    m_isJetpackEnabled = m_savedJetpackEnabled;
    std::cout << "Docking released: free movement" << std::endl;
}

void DigibotController::frameUpdate(double deltaTimeRemainder) {
    // Reset to zero at start of frame
    m_surfaceAngularVelocity = glm::dvec3{0.0, 0.0, 0.0};

    // Determine which rigid body to co-rotate the view with
    std::shared_ptr<RigidBody> targetRigidBody{};

    if (m_dockingState != DockingState::FREE) {
        // Docked or seated: co-rotate with the cockpit's grid
        targetRigidBody = m_dockingTarget.m_gridBody.lock();
    } else if (m_isJetpackEnabled) {
        // Flying mode: use FULL_LOCK target body
        if (m_lockState == DigibotLockState::FULL_LOCK) {
            targetRigidBody = m_targetRigidBody.lock();
        }
    } else {
        // Walking mode: use the grid we're standing on
        targetRigidBody = m_walkingMode.getWalkingTargetRigidBody().lock();
    }

    if (targetRigidBody) {
        m_surfaceAngularVelocity = targetRigidBody->getAngularVelocityWorld();
        double angularVelocityMagnitude{glm::length(m_surfaceAngularVelocity)};

        if (angularVelocityMagnitude > 1e-6) {
            double rotationAngle{angularVelocityMagnitude * deltaTimeRemainder};
            glm::dvec3 rotationAxis{m_surfaceAngularVelocity / angularVelocityMagnitude};

            glm::dquat surfaceRotation{glm::angleAxis(rotationAngle, rotationAxis)};

            m_viewDirection = surfaceRotation * m_viewDirection;
            m_viewDirection = glm::normalize(m_viewDirection);
        }
    }

    // Clamp view direction to prevent near-vertical angles
    auto rigidBody = m_physics->getRigidBody().lock();
    if (!rigidBody) {
        return;
    }

    glm::dvec3 bodyUp{rigidBody->m_orientation * glm::dvec3{0.0, 0.0, 1.0}};

    // Decompose view direction into planar and up components
    glm::dvec3 upComponent{glm::dot(m_viewDirection, bodyUp) * bodyUp};
    glm::dvec3 planarComponent{m_viewDirection - upComponent};

    // Calculate minimum planar length for maximum allowed angle
    const double minAngle{glm::radians(5.0)};
    const double minPlanarLength{glm::sin(minAngle)};

    double planarLengthSq{glm::length2(planarComponent)};

    if (planarLengthSq < minPlanarLength * minPlanarLength) {
        // Handle edge case: if planar is near-zero, add small offset
        double planarLength{};
        if (planarLengthSq < 1e-9) {
            glm::dvec3 bodyForward{rigidBody->m_orientation * glm::dvec3{0.0, 1.0, 0.0}};
            planarComponent += 0.01 * bodyForward;
            planarLength = glm::length(planarComponent);
        } else {
            planarLength = glm::sqrt(planarLengthSq);
        }

        // Scale planar component to minimum length
        planarComponent = (planarComponent / planarLength) * minPlanarLength;

        // Reconstruct and normalize
        m_viewDirection = glm::normalize(planarComponent + upComponent);
    }
}

void DigibotController::applyWrench(const std::weak_ptr<RigidBody>& bodyWeak,
                                    const std::shared_ptr<RigidBody>& body,
                                    const DigibotWrench& wrench, double scale) {
    assert(body && "applyWrench requires a live body");
    glm::dvec3 force{wrench.m_force * scale};
    glm::dvec3 torque{wrench.m_torque * scale};

    // Central NaN net: any mode producing a non-finite wrench (bad normalize, div by
    // zero, degenerate inertia) is caught here before it poisons the rigid body and
    // silently corrupts the whole simulation.
    assert(std::isfinite(force.x) && std::isfinite(force.y) && std::isfinite(force.z) &&
           std::isfinite(torque.x) && std::isfinite(torque.y) && std::isfinite(torque.z) &&
           "DigibotController::applyWrench: non-finite wrench from a movement mode");

    if (glm::length2(force) > 0.0) {
        m_physicsEngine->applyForce(bodyWeak, force);
    }
    if (glm::length2(torque) > 0.0) {
        m_physicsEngine->applyTorque(bodyWeak, torque);
    }

    // Equal and opposite reaction so momentum and angular momentum are conserved.
    if (!wrench.m_reactionBody.expired()) {
        m_physicsEngine->applyForceAtPoint(wrench.m_reactionBody, -force,
                                           body->m_position);
        m_physicsEngine->applyTorque(wrench.m_reactionBody, -torque);
    }
}

void DigibotController::stepControl() {
    auto rigidBodyWeak = m_physics->getRigidBody();
    auto rigidBody = rigidBodyWeak.lock();
    if (!rigidBody) {
        return;
    }

    // Drop an expired lock target before the modes see it
    if (m_lockState != DigibotLockState::UNLOCKED && m_targetRigidBody.expired()) {
        std::cout << "Target rigid body destroyed - unlocking" << std::endl;
        unlock();
    }

    DigibotModeInputs inputs{m_movementDirection, m_viewDirection, m_rollInput,
                             m_isUpDirectionLocked};

    // The docking state fully selects the force logic - no blending. DOCKED and
    // SEATED run only the cockpit modes; FREE runs flying or walking.
    if (m_dockingState == DockingState::SEATED) {
        auto gridBody = m_dockingTarget.m_gridBody.lock();
        if (!gridBody) {
            releaseDocking();
        } else {
            DigibotDockingMode::Result result{
                m_dockingMode.updateSeated(rigidBody, gridBody, m_dockingTarget)};
            applyWrench(rigidBodyWeak, rigidBody, result.m_wrench, 1.0);
            if (result.m_wantRelease) {
                // Restraint overwhelmed - thrown back into the corridor
                m_dockingState = DockingState::DOCKED;
                m_dockingMode.disarmSeatCapture();
            }
            return;
        }
    } else if (m_dockingState == DockingState::DOCKED) {
        auto gridBody = m_dockingTarget.m_gridBody.lock();
        if (!gridBody) {
            releaseDocking();
        } else {
            DigibotDockingMode::Result result{m_dockingMode.updateDocked(
                rigidBody, gridBody, m_dockingTarget, inputs)};
            applyWrench(rigidBodyWeak, rigidBody, result.m_wrench, 1.0);
            if (result.m_wantRelease) {
                releaseDocking();
            } else if (result.m_wantSeat) {
                m_dockingState = DockingState::SEATED;
                std::cout << "Seated in cockpit" << std::endl;
            }
            return;
        }
    }

    // ========== Free movement ==========
    DigibotWrench freeWrench{};
    if (m_isJetpackEnabled) {
        freeWrench = m_flyingMode.update(rigidBody, inputs, m_lockState,
                                         m_targetRigidBody.lock());
    } else {
        freeWrench = m_walkingMode.update(rigidBody, inputs);
    }
    applyWrench(rigidBodyWeak, rigidBody, freeWrench, 1.0);
}
