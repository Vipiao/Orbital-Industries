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
    , m_walkingNoiseCounter(0)
    , m_hitRatio(0.0)
    , m_averageNormal(0.0, 0.0, 1.0)
    , m_averageGroundDistance(0.0)
    , m_lastHitPoint(0.0, 0.0, 0.0)
    , m_runningAverageAlpha(0.2)
    , m_hitRatioThreshold(0.5)
    , m_gridWeightRemovalThreshold(0.01)
    , m_maxGroundAcceleration(0.004)
    , m_targetHoverHeight(1.0)
    , m_minGridMassFraction(0.5)
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
    double lockForceScale = 0.2;

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

    // Get grids from walking sensor
    std::vector<std::weak_ptr<Grid>> availableGrids;
    if (auto sensor = m_physics->getWalkingSensor().lock()) {
        SensorCollider* sensorPtr = static_cast<SensorCollider*>(sensor.get());
        availableGrids = m_gridSubsystem->getGridsFromOverlaps(sensorPtr);
    }

    // Generate 2D noise for ray direction perturbation
    double noiseX = Hash::pcgUnit(m_walkingNoiseCounter++) * 0.6 - 0.3;
    double noiseY = Hash::pcgUnit(m_walkingNoiseCounter++) * 0.6 - 0.3;

    noiseX = 0.;
    noiseY = 0.;
    
    // Create local ray direction: downward (-Z) with XY noise
    glm::dvec3 localRayDir = glm::normalize(glm::dvec3(noiseX, noiseY, -1.0));
    
    // Transform to world space
    glm::dvec3 worldRayDir = rigidBody->m_orientation * localRayDir;
    
    // Ray starts at rigid body center
    glm::dvec3 rayStart = rigidBody->m_position;
    glm::dvec3 rayEnd = rayStart + worldRayDir * 10.0; // 10 meter ray length
    
    // Perform raycasting against available grids
    bool hitThisFrame = false;
    double closestT = -1.0;
    std::weak_ptr<Grid> hitGrid;
    glm::dvec3 hitNormal(0.0, 0.0, 0.0);
    glm::dvec3 hitPoint(0.0, 0.0, 0.0);
    
    for (const auto& gridWeak : availableGrids) {
        auto gridShared = gridWeak.lock();
        if (!gridShared) continue;
        
        // Transform ray to grid-local space
        glm::dvec3 gridLocalRayStart = gridShared->worldToGrid(rayStart);
        glm::dvec3 gridLocalRayEnd = gridShared->worldToGrid(rayEnd);
        
        // Perform ray intersection
        RayIntersectionResult result = gridShared->intersectRay(gridLocalRayStart, gridLocalRayEnd);
        
        // Check if this is a closer hit
        if (result.t >= 0.0 && (!hitThisFrame || result.t < closestT)) {
            closestT = result.t;
            hitThisFrame = true;
            hitGrid = gridWeak;
            
            // Transform normal back to world space
            hitNormal = glm::normalize(gridShared->getRigidBody()->m_orientation * result.surfaceNormal);
            
            // Calculate hit point in world space
            hitPoint = rayStart + closestT * (rayEnd - rayStart);
        }
    }
    
    // Update hit ratio using exponential moving average
    double hitFlag = hitThisFrame ? 1.0 : 0.0;
    m_hitRatio = glm::mix(m_hitRatio, hitFlag, m_runningAverageAlpha);
    
    // Get body up vector for hovering calculations
    glm::dvec3 bodyUpVector = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
    
    // Update running averages if we hit something
    if (hitThisFrame) {
        // Update average normal
        m_averageNormal = glm::normalize(glm::mix(m_averageNormal, hitNormal, m_runningAverageAlpha));
        
        // Calculate projected distance along body vertical direction
        double projectedDistance = -glm::dot(hitPoint - rigidBody->m_position, bodyUpVector);
        
        // Update average ground distance (projected)
        m_averageGroundDistance = glm::mix(m_averageGroundDistance, projectedDistance, m_runningAverageAlpha);
        
        // Store last hit point for force calculation
        m_lastHitPoint = hitPoint;
        
        // Update grid weights
        auto hitGridShared = hitGrid.lock();
        if (hitGridShared) {
            bool foundGrid = false;
            for (size_t i = 0; i < m_groundGrids.size(); ++i) {
                auto gridPtr = m_groundGrids[i].lock();
                if (gridPtr && gridPtr->uniqueId == hitGridShared->uniqueId) {
                    // Grid already tracked - update weight towards 1.0
                    m_gridWeights[i] = glm::mix(m_gridWeights[i], 1.0, m_runningAverageAlpha);
                    foundGrid = true;
                    break;
                }
            }
            
            if (!foundGrid) {
                // New grid - add to tracking
                m_groundGrids.push_back(hitGrid);
                m_gridWeights.push_back(m_runningAverageAlpha);
            }
        }
    }
    
    // Decay weights for all grids not hit this frame
    for (size_t i = 0; i < m_groundGrids.size(); ++i) {
        auto gridPtr = m_groundGrids[i].lock();
        if (!gridPtr || (hitThisFrame && hitGrid.lock() && gridPtr->uniqueId == hitGrid.lock()->uniqueId)) {
            continue; // Skip the grid we just hit or expired grids
        }
        
        // Decay weight towards 0.0
        m_gridWeights[i] = glm::mix(m_gridWeights[i], 0.0, m_runningAverageAlpha);
    }
    
    // Remove grids with weight below threshold (iterate backwards for safe removal)
    for (int i = static_cast<int>(m_groundGrids.size()) - 1; i >= 0; --i) {
        if (m_gridWeights[i] < m_gridWeightRemovalThreshold || m_groundGrids[i].expired()) {
            m_groundGrids.erase(m_groundGrids.begin() + i);
            m_gridWeights.erase(m_gridWeights.begin() + i);
        }
    }
    
    // Print debug info periodically (every 60 frames)
    if (m_walkingNoiseCounter % 60 == 0) {
        std::cout << "Walking: hitRatio=" << m_hitRatio << " grids: ";
        for (size_t i = 0; i < m_groundGrids.size(); ++i) {
            auto gridPtr = m_groundGrids[i].lock();
            std::cout << (gridPtr ? gridPtr->uniqueId : 0) << ":" << m_gridWeights[i] << " ";
        }
        std::cout << std::endl;
    }

    // Apply hovering force if hit ratio is above threshold
    if (m_hitRatio >= m_hitRatioThreshold) {
        // Calculate instantaneous distance from rigid body to last hit point along average normal
        double projectedDistance = -glm::dot(m_lastHitPoint - rigidBody->m_position, m_averageNormal);
        
        // Calculate distance error
        double distanceError = m_targetHoverHeight - projectedDistance;
        
        // Calculate target velocity with safety margin
        double margin = 0.5;
        double targetSpeed = glm::sqrt(2.0 * m_maxGroundAcceleration * (1.0 - margin) * glm::abs(distanceError));
        double targetVelocity = (distanceError > 0.0 ? targetSpeed : -targetSpeed);
        
        // Current velocity in average normal direction
        double currentVelocity = glm::dot(rigidBody->m_velocity, m_averageNormal);
        
        // Needed acceleration
        double neededAcceleration = targetVelocity - currentVelocity;
        neededAcceleration = glm::clamp(neededAcceleration, -m_maxGroundAcceleration, m_maxGroundAcceleration);
        
        // Force on digibot
        glm::dvec3 hoverForce = neededAcceleration * rigidBody->m_mass * m_averageNormal;
        m_physicsEngine->applyForce(rigidBody, hoverForce);
        
        // Calculate total weight for normalization
        double totalWeight = 0.0;
        for (double weight : m_gridWeights) {
            totalWeight += weight;
        }
        
        // Apply opposite forces to grids
        if (totalWeight > 0.0) {
            for (size_t i = 0; i < m_groundGrids.size(); ++i) {
                auto gridPtr = m_groundGrids[i].lock();
                if (!gridPtr) continue;
                
                RigidBody* gridBody = gridPtr->getRigidBody();
                if (!gridBody) continue;
                
                // Check mass threshold - skip if grid is too light to stand on
                if (gridBody->m_mass < m_minGridMassFraction * rigidBody->m_mass) {
                    continue;
                }
                
                // Apply weighted opposite force at digibot rigid body center
                double weightFraction = m_gridWeights[i] / totalWeight;
                glm::dvec3 gridForce = -hoverForce * weightFraction;
                
                // Apply force at digibot center position (same point for both bodies)
                m_physicsEngine->applyForceAtPoint(gridBody, gridForce, rigidBody->m_position);
            }
        }
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
    
    // Adjust max angular velocity down when close to final target to be more precise
    double adjustedAngVelMax = m_angularAccelerationMax * glm::abs(deltaAngle);
    
    // Calculate target angular velocity if we need to rotate
    if (axisLength > 1e-6) {
        // Normalize the rotation axis
        rotationAxis = rotationAxis / axisLength;
        
        // 3. Calculate maximum angular speed based on remaining angle and max acceleration
        double margin = 0.2;
        double maxAngularSpeed = std::sqrt(2.0 * adjustedAngVelMax * (1.0 - margin) * deltaAngle);
        
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

    // Apply movement force
    if (m_movementDirection != glm::ivec3(0, 0, 0)) {
        // Get body's upward direction
        glm::dvec3 bodyUpDirection = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
        
        // Create view orientation quaternion using view direction and body's up vector
        glm::dquat viewOrientation = glm::conjugate(ArticulationUtils::quatLookAtYForward(m_viewDirection, bodyUpDirection));
        
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