// DigibotController.cpp
#include "DigibotController.h"
#include "DigibotPhysics.h"
#include "../../physics/RigidBody.h"
#include "../../physics/PhysicsEngine.h"
#include "../../game_base/Grid.h"
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include "../ArticulationUtils.h"
#include "../../game_base/GridSubsystem.h"
#include "../../physics/SensorCollider.h"
#include "../../utils/GridGeometry.h"

DigibotController::DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem)
    : m_physics(physics)
    , m_physicsEngine(physicsEngine)
    , m_gridSubsystem(gridSubsystem)
    , m_movementDirection(0, 0, 0)
    , m_rollInput(0)
    , m_thrustStrength(0.004)  // Default thrust strength
    , m_angularAccelerationMax(0.004)  // Maximum angular acceleration (rad/s²)
    , m_rollAcceleration(0.005)  // Roll acceleration strength (rad/s^2)
    , m_viewDirection(0.0, 1.0, 0.0)  // Default forward
    , m_lockState(LockState::UNLOCKED)
    , m_hasCachedPoint(false)
    , m_cachedLocalPoint(0.0, 0.0, 0.0)
    , m_cachedLocalNormal(0.0, 0.0, 1.0)
    , m_targetHoverHeight(1.0)
    , m_maxGroundAcceleration(0.004)
    , m_walkingRayLength(1.4)
    , m_jetpackEnabled(true)
    , m_translationLockStrength(1.0)
{
    if (!m_physics) {
        throw std::runtime_error("DigibotController: Physics component cannot be null");
    }
    
    if (!m_physicsEngine) {
        throw std::runtime_error("DigibotController: Physics engine cannot be null");
    }

    if (!m_gridSubsystem) {
        throw std::runtime_error("DigibotController: Grid subsystem cannot be null");
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
    m_jetpackEnabled = enabled;
    std::cout << "Jetpack " << (m_jetpackEnabled ? "ENABLED" : "DISABLED") << std::endl;
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
    if (m_jetpackEnabled) {
        handleFlying();
    } else {
        handleWalking();
    }
}

void DigibotController::handleFlying() {
    // Get the rigid body from physics component
    RigidBody* rigidBody = m_physics->getRigidBody();
    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return;
    }

    // Lock force scaling parameter
    double lockForceScale = 0.4;

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

                // Cap the lock force if above limit
                double lockForceMagnitude = glm::length(lockForce);
                double lockForceLimit = lockForceScale * m_thrustStrength * rigidBody->m_mass;
                if (lockForceMagnitude > lockForceLimit) {
                    lockForce = lockForce * (lockForceLimit / lockForceMagnitude);
                }
            }
        }
    } else if (m_lockState == LockState::FULL_LOCK) {
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
                // ========== Translation Component ==========
                // Get character position
                glm::dvec3 characterPos = rigidBody->m_position;
                
                // Get grid's center of mass position
                glm::dvec3 gridPos = targetGridRigidBody->m_position;
                
                // Calculate radius vector from grid center to character
                glm::dvec3 radiusVector = characterPos - gridPos;
                
                // Get grid's angular velocity
                glm::dvec3 gridAngularVelocity = targetGridRigidBody->getAngularVelocityWorld();
                
                // Calculate velocity at character position due to grid rotation: v = ω × r
                glm::dvec3 velocityFromRotation = glm::cross(gridAngularVelocity, radiusVector);
                
                // Total target linear velocity = grid's linear velocity + velocity from rotation
                glm::dvec3 targetLinearVelocity = targetGridRigidBody->m_velocity + velocityFromRotation;
                
                // Calculate relative velocity
                glm::dvec3 digibotVelocity = rigidBody->m_velocity;
                glm::dvec3 relativeLinearVelocity = digibotVelocity - targetLinearVelocity;
                
                // Calculate correction force
                glm::dvec3 correctionForce = -relativeLinearVelocity * m_translationLockStrength * rigidBody->m_mass;
                
                // Project correction force to plane orthogonal to movement force
                if (glm::length(movementForce) > 1e-6) {
                    glm::dvec3 movementDirection = glm::normalize(movementForce);
                    double projectionOntoMovement = glm::dot(correctionForce, movementDirection);
                    correctionForce = correctionForce - projectionOntoMovement * movementDirection;
                }
                
                lockForce = correctionForce;

                // Cap the lock force if above limit
                double lockForceMagnitude = glm::length(lockForce);
                double lockForceLimit = lockForceScale * m_thrustStrength * rigidBody->m_mass;
                if (lockForceMagnitude > lockForceLimit) {
                    lockForce = lockForce * (lockForceLimit / lockForceMagnitude);
                }

                // Add compensation for centrifugal and coriolis forces
                glm::dvec3 centrifugalForce = -rigidBody->m_mass * 
                    glm::cross(gridAngularVelocity, glm::cross(gridAngularVelocity, radiusVector));
                glm::dvec3 coriolisForce = -2.0 * rigidBody->m_mass * 
                    glm::cross(gridAngularVelocity, relativeLinearVelocity);
                glm::dvec3 compensationForce = -centrifugalForce - coriolisForce;
                
                lockForce += compensationForce;
            }
        }
    }
    
    // ========== Combine and Clamp Forces ==========
    double maxForce = m_thrustStrength * rigidBody->m_mass;
    
    glm::dvec3 totalForce = movementForce + lockForce;
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
    
    // ========== Add Grid Angular Velocity for Full Lock ==========
    if (m_lockState == LockState::FULL_LOCK && !m_targetGrid.expired()) {
        auto targetGrid = m_targetGrid.lock();
        if (targetGrid) {
            RigidBody* targetGridRigidBody = targetGrid->getRigidBody();
            if (targetGridRigidBody) {
                // Add grid's angular velocity to our target
                targetAngularVelocity += targetGridRigidBody->getAngularVelocityWorld();
            }
        }
    }

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

void DigibotController::handleWalking() {
    // Get the rigid body from physics component
    RigidBody* rigidBody = m_physics->getRigidBody();
    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return;
    }

    // ========== Step 1: Raycast Straight Down ==========
    std::vector<std::weak_ptr<Grid>> availableGrids;
    if (auto sensor = m_physics->getWalkingSensor().lock()) {
        SensorCollider* sensorPtr = static_cast<SensorCollider*>(sensor.get());
        availableGrids = m_gridSubsystem->getGridsFromOverlaps(sensorPtr);
    }

    // Ray straight down in world space
    glm::dvec3 bodyUpVector = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
    glm::dvec3 worldRayDir = -bodyUpVector;
    
    glm::dvec3 rayStart = rigidBody->m_position;
    glm::dvec3 rayEnd = rayStart + worldRayDir * m_walkingRayLength;
    
    bool hitThisFrame = false;
    double closestT = -1.0;
    std::weak_ptr<Grid> hitGrid;
    glm::dvec3 hitNormal(0.0, 0.0, 0.0);
    glm::dvec3 hitPoint(0.0, 0.0, 0.0);
    
    // Raycast against all grids in sensor range
    for (const auto& gridWeak : availableGrids) {
        auto gridShared = gridWeak.lock();
        if (!gridShared) continue;
        
        // Skip grids with insufficient mass
        RigidBody* gridBody = gridShared->getRigidBody();
        if (!gridBody || gridBody->m_mass < 1.0 * rigidBody->m_mass) {
            continue;
        }
        
        // Transform ray to grid-local space
        glm::dvec3 gridLocalRayStart = gridShared->worldToGrid(rayStart);
        glm::dvec3 gridLocalRayEnd = gridShared->worldToGrid(rayEnd);
        
        RayIntersectionResult result = gridShared->intersectRay(gridLocalRayStart, gridLocalRayEnd);
        
        if (result.t >= 0.0 && (!hitThisFrame || result.t < closestT)) {
            closestT = result.t;
            hitThisFrame = true;
            hitGrid = gridWeak;
            
            hitNormal = glm::normalize(gridShared->getRigidBody()->m_orientation * result.surfaceNormal);
            hitPoint = rayStart + closestT * (rayEnd - rayStart);
        }
    }
    
    // ========== Step 2: Cache or Reuse Point ==========
    bool hasSurfaceNormal = false;
    glm::dvec3 surfaceNormal(0.0, 0.0, 1.0);
    if (hitThisFrame) {
        // Store in grid's local coordinate system
        auto hitGridShared = hitGrid.lock();
        if (hitGridShared) {
            m_cachedGrid = hitGrid;
            m_cachedLocalPoint = hitGridShared->worldToGrid(hitPoint);
            m_cachedLocalNormal = glm::normalize(
                glm::conjugate(hitGridShared->getRigidBody()->m_orientation) * hitNormal
            );
            m_hasCachedPoint = true;

            surfaceNormal = hitNormal;
            hasSurfaceNormal = true;
        }
    } else if (m_hasCachedPoint) {
        // Check if cached grid is still valid and in range
        auto cachedGridShared = m_cachedGrid.lock();
        if (!cachedGridShared) {
            m_hasCachedPoint = false;
        } else {
            // Check if cached point is still in sensor range
            glm::dvec3 cachedWorldPoint = cachedGridShared->gridToWorld(m_cachedLocalPoint);
            double distanceToCache = glm::length(cachedWorldPoint - rigidBody->m_position);
            
            if (distanceToCache > m_walkingRayLength) {
                m_hasCachedPoint = false;
                m_cachedGrid.reset();
            } else {
                // Cache is valid - extract surface normal
                RigidBody* gridBody = cachedGridShared->getRigidBody();
                surfaceNormal = glm::normalize(gridBody->m_orientation * m_cachedLocalNormal);
                hasSurfaceNormal = true;
            }
        }
    }
    
    // ========== Step 3: Position Control with Grid Velocity ==========
    if (m_hasCachedPoint) {
        auto cachedGridShared = m_cachedGrid.lock();
        if (cachedGridShared) {
            RigidBody* gridBody = cachedGridShared->getRigidBody();
            if (gridBody) {
                // Transform cached point and normal to world space
                glm::dvec3 cachedWorldPoint = cachedGridShared->gridToWorld(m_cachedLocalPoint);
                glm::dvec3 cachedWorldNormal = glm::normalize(gridBody->m_orientation * m_cachedLocalNormal);
                
                // Calculate target position
                glm::dvec3 targetPosition = cachedWorldPoint + cachedWorldNormal * m_targetHoverHeight;
                
                // Calculate position error
                glm::dvec3 positionError = targetPosition - rigidBody->m_position;
                double distance = glm::length(positionError);
                
                // Calculate target speed
                double margin = 0.5;
                double targetSpeed = std::sqrt(2.0 * m_maxGroundAcceleration * (1.0 - margin) * distance);
                glm::dvec3 targetVelocityDirection = (distance > 1e-6) ? (positionError / distance) : glm::dvec3(0.0);
                
                // Add grid velocity at contact point
                glm::dvec3 radiusVector = cachedWorldPoint - gridBody->m_position;
                glm::dvec3 gridVelocityAtPoint = gridBody->m_velocity + 
                    glm::cross(gridBody->getAngularVelocityWorld(), radiusVector);
                
                glm::dvec3 targetVelocity = targetVelocityDirection * targetSpeed + gridVelocityAtPoint;
                
                // Calculate needed acceleration
                glm::dvec3 velocityError = targetVelocity - rigidBody->m_velocity;
                glm::dvec3 neededAcceleration = velocityError;
                
                // Clamp acceleration
                double accelMagnitude = glm::length(neededAcceleration);
                if (accelMagnitude > m_maxGroundAcceleration) {
                    neededAcceleration = neededAcceleration * (m_maxGroundAcceleration / accelMagnitude);
                }
                
                // Apply force to character
                glm::dvec3 hoverForce = neededAcceleration * rigidBody->m_mass;

                // Reduce force perpendicular to surface normal by 0.1 factor
                glm::dvec3 parallelForce = glm::dot(hoverForce, cachedWorldNormal) * cachedWorldNormal;
                glm::dvec3 perpendicularForce = hoverForce - parallelForce;
                hoverForce = parallelForce + 0.8 * perpendicularForce;

                m_physicsEngine->applyForce(rigidBody, hoverForce);
                
                // Apply opposite force to grid at contact point
                m_physicsEngine->applyForceAtPoint(gridBody, -hoverForce, cachedWorldPoint);
            }
        }
    }
    
    // ========== Step 4: Orient Body to Surface Normal ==========
    if (m_hasCachedPoint) {
        auto cachedGridShared = m_cachedGrid.lock();
        if (cachedGridShared) {
            RigidBody* gridBody = cachedGridShared->getRigidBody();
            if (gridBody) {
                glm::dvec3 cachedWorldNormal = glm::normalize(gridBody->m_orientation * m_cachedLocalNormal);
                
                glm::dvec3 currentUpVector = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
                currentUpVector = glm::normalize(currentUpVector);
                
                glm::dvec3 targetUpVector = cachedWorldNormal;
                
                double upDotProduct = glm::clamp(glm::dot(currentUpVector, targetUpVector), -1.0, 1.0);
                double upDeltaAngle = std::acos(upDotProduct);
                
                glm::dvec3 upRotationAxis = glm::cross(currentUpVector, targetUpVector);
                double upAxisLength = glm::length(upRotationAxis);
                
                glm::dvec3 upTargetAngularVelocity(0.0, 0.0, 0.0);
                double upAdjustedAngVelMax = m_angularAccelerationMax * glm::abs(upDeltaAngle);
                
                if (upAxisLength > 1e-6) {
                    upRotationAxis = upRotationAxis / upAxisLength;
                    double upMargin = 0.2;
                    double upMaxAngularSpeed = std::sqrt(2.0 * upAdjustedAngVelMax * (1.0 - upMargin) * upDeltaAngle);
                    upTargetAngularVelocity = upRotationAxis * upMaxAngularSpeed;
                }
                
                // Apply up-vector alignment torque
                glm::dvec3 currentAngularVelocity = rigidBody->getAngularVelocityWorld();
                glm::dvec3 angularAcceleration = upTargetAngularVelocity - currentAngularVelocity;
                
                // Remove any component around the body's up axis (z-axis)
                glm::dvec3 bodyUpAxis = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
                glm::dvec3 aroundUpComponent = glm::dot(angularAcceleration, bodyUpAxis) * bodyUpAxis;
                angularAcceleration = angularAcceleration - aroundUpComponent;

                double angAccMagnitude = glm::length(angularAcceleration);
                if (angAccMagnitude > m_angularAccelerationMax) {
                    angularAcceleration = angularAcceleration * (m_angularAccelerationMax / angAccMagnitude);
                }
                
                m_physicsEngine->applyTorque(rigidBody, rigidBody->getWorldInertiaTensor() * angularAcceleration);
            }
        }
    }
    
    // ========== Step 5: View Direction Rotation ==========
    glm::dvec3 currentForward = rigidBody->m_orientation * glm::dvec3(0.0, 1.0, 0.0);
    currentForward = glm::normalize(currentForward);
    
    glm::dvec3 targetForward = glm::normalize(m_viewDirection);
    
    double forwardDotProduct = glm::clamp(glm::dot(currentForward, targetForward), -1.0, 1.0);
    double forwardDeltaAngle = std::acos(forwardDotProduct);
    
    glm::dvec3 forwardRotationAxis = glm::cross(currentForward, targetForward);
    double forwardAxisLength = glm::length(forwardRotationAxis);
    
    glm::dvec3 forwardTargetAngularVelocity(0.0, 0.0, 0.0);
    double forwardAdjustedAngVelMax = m_angularAccelerationMax * glm::abs(forwardDeltaAngle);
    
    if (forwardAxisLength > 1e-6) {
        forwardRotationAxis = forwardRotationAxis / forwardAxisLength;
        double forwardMargin = 0.2;
        double forwardMaxAngularSpeed = std::sqrt(2.0 * forwardAdjustedAngVelMax * (1.0 - forwardMargin) * forwardDeltaAngle);
        forwardTargetAngularVelocity = forwardRotationAxis * forwardMaxAngularSpeed;
    }
    
    // Apply view direction torque
    glm::dvec3 currentAngularVelocity = rigidBody->getAngularVelocityWorld();
    glm::dvec3 angularAcceleration = forwardTargetAngularVelocity - currentAngularVelocity;
    
    // Keep only rotation around the surface normal if we have one
    if (hasSurfaceNormal) {
        glm::dvec3 parallelComponent = glm::dot(angularAcceleration, surfaceNormal) * surfaceNormal;
        angularAcceleration = parallelComponent;
    }

    double angAccMagnitude = glm::length(angularAcceleration);
    if (angAccMagnitude > m_angularAccelerationMax) {
        angularAcceleration = angularAcceleration * (m_angularAccelerationMax / angAccMagnitude);
    }
    
    m_physicsEngine->applyTorque(rigidBody, rigidBody->getWorldInertiaTensor() * angularAcceleration);

    // ========== Step 6: Apply Movement Force ==========
    if (m_movementDirection != glm::ivec3(0, 0, 0)) {
        glm::dquat viewOrientation = glm::conjugate(
            ArticulationUtils::quatLookAtYForward(m_viewDirection, bodyUpVector)
        );
        
        glm::dvec3 direction = glm::dvec3(
            static_cast<double>(m_movementDirection.x),
            static_cast<double>(m_movementDirection.y),
            static_cast<double>(m_movementDirection.z)
        );
        
        if (glm::length(direction) > 0.0) {
            direction = glm::normalize(direction);
            direction = viewOrientation * direction;
            
            double forceMagnitude = m_thrustStrength * rigidBody->m_mass;
            glm::dvec3 movementForce = direction * forceMagnitude;
            m_physicsEngine->applyForce(rigidBody, movementForce);
        }
    }
}

void DigibotController::setThrustStrength(double strength) {
    m_thrustStrength = strength;
}

void DigibotController::setRollAcceleration(double acceleration) {
    m_rollAcceleration = acceleration;
}