// DigibotController.cpp
#include "DigibotController.h"
#include "DigibotPhysics.h"
#include "../../physics/RigidBody.h"
#include "../../physics/PhysicsEngine.h"
#include "../../game_base/Grid.h"
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include "../ArticulationUtils.h"

DigibotController::DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine)
    : m_physics(physics)
    , m_physicsEngine(physicsEngine)
    , m_movementDirection(0, 0, 0)
    , m_rollInput(0)
    , m_thrustStrength(0.004)  // Default thrust strength
    , m_angularAccelerationMax(0.004)  // Maximum angular acceleration (rad/s²)
    , m_rollAcceleration(0.005)  // Roll acceleration strength (rad/s^2)
    , m_viewDirection(0.0, 1.0, 0.0)  // Default forward
    , m_lockState(LockState::UNLOCKED)
    , m_translationLockStrength(1.0)
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

void DigibotController::setTargetGrid(std::weak_ptr<Grid> grid) {
    m_targetGrid = grid;
    std::cout << "Target grid set" << std::endl;
}

void DigibotController::unlock() {
    m_targetGrid.reset();
    m_lockState = LockState::UNLOCKED;
    std::cout << "Unlocked from grid" << std::endl;
}

void DigibotController::physics() {
    // Get the rigid body from physics component
    RigidBody* rigidBody = m_physics->getRigidBody();
    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return;
    }

    // ========== Calculate View Orientation ==========
    // Get body's upward direction
    glm::dvec3 bodyUpDirection = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
    
    // Create view orientation quaternion using view direction and body's up vector 
    // Note: We need to conjugate/invert the quaternion to transform vectors correctly
    glm::dquat viewOrientation = glm::conjugate(ArticulationUtils::quatLookAtYForward(m_viewDirection, bodyUpDirection));
    
    // ========== Calculate Movement Force ==========
    glm::dvec3 movementForce(0.0, 0.0, 0.0);
    if (m_movementDirection != glm::ivec3(0, 0, 0)) {
        // Convert integer direction to normalized 3D vector
        glm::dvec3 direction = glm::dvec3(
            static_cast<double>(m_movementDirection.x),
            static_cast<double>(m_movementDirection.y),
            static_cast<double>(m_movementDirection.z)
        );
        
        // Normalize if not zero
        if (glm::length(direction) > 0.0) {
            direction = glm::normalize(direction);

            // Transform direction from local to world space using view orientation
            direction = viewOrientation * direction;
            
            // Calculate thrust force based on mass
            double forceMagnitude = m_thrustStrength * rigidBody->m_mass;
            movementForce = direction * forceMagnitude;
        }
    }
    
    // ========== Handle Translation Lock ==========
    glm::dvec3 lockForce(0.0, 0.0, 0.0);
    if (m_lockState == LockState::TRANSLATION_LOCK) {
        // Check if target grid is still valid
        if (m_targetGrid.expired()) {
            std::cout << "Target grid destroyed - unlocking" << std::endl;
            unlock();
        } else {
            // Get target grid's rigid body
            auto targetGrid = m_targetGrid.lock();
            RigidBody* targetGridRigidBody = targetGrid->getRigidBody();
            
            if (!targetGridRigidBody) {
                std::cout << "Target grid has no rigid body - unlocking" << std::endl;
                unlock();
            } else {
                // Calculate relative velocity
                glm::dvec3 digibotVelocity = rigidBody->m_velocity;
                glm::dvec3 gridVelocity = targetGridRigidBody->m_velocity;
                glm::dvec3 relativeVelocity = digibotVelocity - gridVelocity;
                
                // Calculate correction force
                glm::dvec3 correctionForce = -relativeVelocity * m_translationLockStrength * rigidBody->m_mass;
                
                // Project correction force to plane orthogonal to movement force
                if (glm::length(movementForce) > 1e-6) {
                    glm::dvec3 movementDirection = glm::normalize(movementForce);
                    double projectionOntoMovement = glm::dot(correctionForce, movementDirection);
                    correctionForce = correctionForce - projectionOntoMovement * movementDirection;
                }
                
                lockForce = correctionForce;
            }
        }
    } else if (m_lockState == LockState::FULL_LOCK) {
        // Check if target grid is still valid
        if (m_targetGrid.expired()) {
            std::cout << "Target grid destroyed - unlocking" << std::endl;
            unlock();
        } else {
            // TODOO: Implement full lock (translation + rotation)
        }
    }
    
    // ========== Combine and Clamp Forces ==========
    glm::dvec3 totalForce = movementForce + lockForce;
    double maxForce = m_thrustStrength * rigidBody->m_mass;
    double totalForceMagnitude = glm::length(totalForce);
    
    if (totalForceMagnitude > maxForce) {
        totalForce = totalForce * (maxForce / totalForceMagnitude);
    }
    
    // Apply combined force at center of mass
    if (totalForceMagnitude > 1e-6) {
        m_physicsEngine->applyForce(rigidBody, totalForce);
    }

    // ========== Handle Roll Input ==========
    if (m_rollInput != 0) {
        // Calculate roll torque around view direction
        glm::dvec3 rollAxis = glm::normalize(m_viewDirection);

        double adjustedRollAcceleration = m_rollAcceleration;
        if(m_rollInput != 0) {
            glm::dvec3 currentAngVel = rigidBody->getAngularVelocityWorld();
            double currentRollRate = glm::dot(currentAngVel, rollAxis);
            if (glm::abs(currentRollRate) < 0.01)
            {
                adjustedRollAcceleration *= 4.;
            }
        }
        
        // Scale by roll acceleration, inertia, and input direction
        double torqueMagnitude = adjustedRollAcceleration * static_cast<double>(m_rollInput);
        glm::dvec3 rollTorque = rollAxis * torqueMagnitude;
        m_physicsEngine->applyTorque(rigidBody, rigidBody->getWorldInertiaTensor() * rollTorque);
    }

    // ========== View Direction Rotation Logic ==========
    // 1. Calculate local forward direction in world space
    glm::dvec3 currentForward = rigidBody->m_orientation * glm::dvec3(0.0, 1.0, 0.0);
    currentForward = glm::normalize(currentForward);
    
    // 2. Calculate angle between current forward and desired view direction
    glm::dvec3 targetForward = glm::normalize(m_viewDirection);
    
    double dotProduct = glm::clamp(glm::dot(currentForward, targetForward), -1.0, 1.0);
    double deltaAngle = std::acos(dotProduct);
    
    // Get rotation axis using cross product
    glm::dvec3 rotationAxis = glm::cross(currentForward, targetForward);
    double axisLength = glm::length(rotationAxis);
    
    // Initialize target angular velocity
    glm::dvec3 targetAngularVelocity(0.0, 0.0, 0.0);
    
    //std::cout << deltaAngle << std::endl;
    // Adjust max angular velocity down when close to final target to be more
    // precise.
    double adjustedAngVelMax =
        m_angularAccelerationMax * glm::abs(deltaAngle);
    // Calculate target angular velocity if we need to rotate
    if (axisLength > 1e-6) {
        // Normalize the rotation axis
        rotationAxis = rotationAxis / axisLength;
        
        // 3. Calculate maximum angular speed based on remaining angle and max acceleration
        double margin = 0.2;
        double maxAngularSpeed = std::sqrt(
            2.0 * adjustedAngVelMax * (1. - margin) * deltaAngle
        );
        
        // 4. Calculate target angular velocity
        targetAngularVelocity = rotationAxis * maxAngularSpeed;
    }
    // else: targetAngularVelocity remains zero - we want to stop any rotation
    
    // 5. Calculate angular acceleration needed (always do this to handle deceleration)
    glm::dvec3 currentAngularVelocity = rigidBody->getAngularVelocityWorld();
    glm::dvec3 angularAcceleration = targetAngularVelocity - currentAngularVelocity;
    
    // 6. Limit acceleration and apply torque
    double angAccMagnitude = glm::length(angularAcceleration);
    if (angAccMagnitude > m_angularAccelerationMax) {
        angularAcceleration = angularAcceleration * (m_angularAccelerationMax / angAccMagnitude);
    }
    
    // Apply torque (I * α = τ)
    m_physicsEngine->applyTorque(rigidBody, rigidBody->getWorldInertiaTensor() * angularAcceleration);
}

void DigibotController::setThrustStrength(double strength) {
    m_thrustStrength = strength;
}

void DigibotController::setRollAcceleration(double acceleration) {
    m_rollAcceleration = acceleration;
}