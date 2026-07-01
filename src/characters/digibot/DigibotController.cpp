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
#include "../../debug/DebugGlobals.h"
#include "../../debug/DebugRenderer.h"

DigibotController::DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem)
    : m_physics(physics)
    , m_physicsEngine(physicsEngine)
    , m_gridSubsystem(gridSubsystem)
    , m_movementDirection(0, 0, 0)
    , m_thrustStrength(0.004)  // Default thrust strength
    , m_angularAccelerationMax(0.016)  // Maximum angular acceleration (rad/s²)
    , m_maxRollRate(0.032)  // Target roll rate (rad/s) - reduced by 0.8x
    , m_rollAcceleration(0.008)  // Roll acceleration strength (rad/s^2)
    , m_rollInput(0)
    , m_viewDirection(0.0, 1.0, 0.0)  // Default forward
    , m_lockState(LockState::UNLOCKED)
    , m_translationLockStrength(1.0)
    , m_jetpackEnabled(true)
    , m_targetHoverHeight(1.0)
    , m_maxGroundAcceleration(0.008)
    , m_targetWalkSpeed(0.08)
    , m_walkingThrustStrength(0.007)
    , m_groundSelectionBias(2.0)
    , m_maxGroundAngle(glm::radians(90.0))
    , m_maxLockedGroundAngle(glm::radians(30.0))
    , m_walkingSensorRadius(physics->getWalkingSensorRadius())
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
    if (enabled) {
        m_cachedRigidBody.reset();
        m_cachedModifiedUp = glm::dvec3(0.0, 0.0, 0.0);
        m_framesWithoutContact = 0;
    }
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

void DigibotController::updatePerFrame(double deltaTimeRemainder) {
    // Reset to zero at start of frame
    m_surfaceAngularVelocity = glm::dvec3(0.0, 0.0, 0.0);
    
    // Determine which rigid body to use for rotation
    std::shared_ptr<RigidBody> targetRigidBody;
    
    if (m_jetpackEnabled) {
        // Flying mode: use FULL_LOCK target grid
        if (m_lockState == LockState::FULL_LOCK) {
            auto targetGrid = m_targetGrid.lock();
            if (targetGrid) {
                targetRigidBody = targetGrid->getRigidBody().lock();
            }
        }
    } else {
        // Walking mode: use the grid we're standing on
        targetRigidBody = m_walkingTargetRigidBody.lock();
    }
    
    if (targetRigidBody) {
        m_surfaceAngularVelocity = targetRigidBody->getAngularVelocityWorld();
        double angularVelocityMagnitude = glm::length(m_surfaceAngularVelocity);
        
        if (angularVelocityMagnitude > 1e-6) {
            double rotationAngle = angularVelocityMagnitude * deltaTimeRemainder;
            glm::dvec3 rotationAxis = m_surfaceAngularVelocity / angularVelocityMagnitude;
            
            glm::dquat surfaceRotation = glm::angleAxis(rotationAngle, rotationAxis);
            
            m_viewDirection = surfaceRotation * m_viewDirection;
            m_viewDirection = glm::normalize(m_viewDirection);
        }
    }

    // Clamp view direction to prevent near-vertical angles
    auto rigidBodyWeak = m_physics->getRigidBody();
    auto rigidBody = rigidBodyWeak.lock();
    if (!rigidBody) {
        return;
    }
    
    glm::dvec3 bodyUp = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
    
    // Decompose view direction into planar and up components
    glm::dvec3 upComponent = glm::dot(m_viewDirection, bodyUp) * bodyUp;
    glm::dvec3 planarComponent = m_viewDirection - upComponent;
    
    // Calculate minimum planar length for maximum allowed angle
    // minAngle = x degrees from vertical
    const double minAngle = glm::radians(5.0);
    const double minPlanarLength = glm::sin(minAngle);
    
    double planarLengthSq = glm::length2(planarComponent);
    
    if (planarLengthSq < minPlanarLength * minPlanarLength) {
        // Handle edge case: if planar is near-zero, add small offset
        double planarLength{};
        if (planarLengthSq < 1e-9) {
            glm::dvec3 bodyForward = rigidBody->m_orientation * glm::dvec3(0.0, 1.0, 0.0);
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

void DigibotController::physics() {
    if (m_jetpackEnabled) {
        handleFlying();
    } else {
        handleWalking();
    }
}

void DigibotController::handleFlying() {
    // Get the rigid body from physics component
    auto rigidBodyWeak = m_physics->getRigidBody();
    auto rigidBody = rigidBodyWeak.lock();
    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return;
    }

    // Lock force scaling parameter
    double lockForceScale = 0.4;

    // ========== Calculate View Orientation ==========
    // Get body's upward direction
    glm::dvec3 bodyUpDirection = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
    
    // Create view orientation quaternion using view direction and body's up vector 
    // Note: We need to conjugate/invert the quaternion as lookAt gives inverse of orientation from vectors.
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
            auto targetGridRigidBodyWeak = targetGrid->getRigidBody();
            auto targetGridRigidBody = targetGridRigidBodyWeak.lock();
            
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
            auto targetGridRigidBodyWeak = targetGrid->getRigidBody();
            auto targetGridRigidBody = targetGridRigidBodyWeak.lock();
            
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
        m_physicsEngine->applyForce(rigidBodyWeak, totalForce);
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
    
    // ========== Add Roll Component to Target Angular Velocity ==========
    // Roll axis is the view direction
    glm::dvec3 rollAxis = glm::normalize(m_viewDirection);
    double targetRollVel = static_cast<double>(m_rollInput) * m_maxRollRate;
    targetAngularVelocity += targetRollVel * rollAxis;

    // ========== Add Grid Angular Velocity for Full Lock ==========
    if (m_lockState == LockState::FULL_LOCK && !m_targetGrid.expired()) {
        auto targetGrid = m_targetGrid.lock();
        if (targetGrid) {
            auto targetGridRigidBodyWeak = targetGrid->getRigidBody();
            auto targetGridRigidBody = targetGridRigidBodyWeak.lock();
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
    m_physicsEngine->applyTorque(rigidBodyWeak, rigidBody->getWorldInertiaTensor() * angularAcceleration);
}

int test = 0;
void DigibotController::handleWalking() {
    test++;
    // DEBUG
    if (DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("closest_contact");
    }

    // Clear walking target at start (will be set if we find ground contact)
    m_walkingTargetRigidBody.reset();

    // Get the rigid body from physics component
    auto rigidBodyWeak = m_physics->getRigidBody();
    auto rigidBody = rigidBodyWeak.lock();
    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return;
    }

    // Calculate reference down direction from body orientation
    glm::dvec3 downDirection = -(rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0));
    
    // Handle cached modified up direction
    glm::dvec3 modifiedUp(0.,0.,0.);
    bool usingCache = false;
    
    if (!m_upDirectionLocked) {
        // Direction is free to change - clear cache
        m_cachedRigidBody.reset();
    } else {
        // Direction is locked - try to use cache
        if (!m_cachedRigidBody.expired()) {
            auto cachedRigidBody = m_cachedRigidBody.lock();
            // Verify cached rigid body still exists
            bool isValid = false;
            for (const auto& weak : m_physicsEngine->getRigidBodies()) {
                if (weak.lock().get() == cachedRigidBody.get()) {
                    isValid = true;
                    break;
                }
            }
            
            if (isValid) {
                // Transform cached direction from rigid body local to world
                modifiedUp = cachedRigidBody->m_orientation * m_cachedModifiedUp;
                usingCache = true;
            } else {
                // Cached rigid body was destroyed - clear cache and recalculate
                m_cachedRigidBody.reset();
            }
        }
    }
    
    // Calculate fresh modifiedUp if needed
    if (!usingCache) {
        glm::dvec3 bodyUpDirection = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
        glm::dvec3 robotRight = rigidBody->m_orientation * glm::dvec3(1.0, 0.0, 0.0);
        modifiedUp = glm::cross(robotRight, m_viewDirection);
        double modifiedUpLengthSq = glm::length2(modifiedUp);
        if (modifiedUpLengthSq < 1e-12) {
            // View direction parallel to robot right - fall back to body up
            modifiedUp = bodyUpDirection;
        } else {
            modifiedUp = modifiedUp / glm::sqrt(modifiedUpLengthSq);
        }
    }

    auto sensor = m_physics->getWalkingSensor().lock();
    if (!sensor) {
        return;
    }
    
    const auto& collisions = m_physicsEngine->getCollisions(sensor.get());
    
    // ========== Step 1: Collect All Contact Candidates ==========
    struct ContactCandidate {
        glm::dvec3 position;
        glm::dvec3 normal;
        double distance;
        std::weak_ptr<RigidBody> rigidBody;
    };
    
    std::vector<ContactCandidate> candidates;
    
    for (const auto& collision : collisions) {
        // Skip collisions with the robot's own body
        ColliderAttachment* otherAttachment = 
            collision.otherCollider->get_pointer<ColliderAttachment>();
        std::weak_ptr<RigidBody> otherRigidBody;
        if (otherAttachment) {
            auto otherRigidBodyShared = otherAttachment->rigidBody.lock();
            if (otherRigidBodyShared && otherRigidBodyShared.get() == rigidBody.get()) {
                continue;
            }
            otherRigidBody = otherAttachment->rigidBody;
        }
        for (const auto& contactPoint : collision.contactPoints) {
            // Calculate surface normal (points from surface toward body)
            glm::dvec3 toBody = rigidBody->m_position - contactPoint;
            double distance = glm::length(toBody);
            
            if (distance < 1e-6) {
                continue; // Skip degenerate case
            }
            
            glm::dvec3 normal = toBody / distance;
            
            candidates.push_back({contactPoint, normal, distance, otherRigidBody});
        }
    }
    
    // ========== Step 2: Plane-Based Filtering ==========
    // Filter out points that are recessed relative to other points
    const double planeBias = 0.01;
    std::vector<bool> filtered(candidates.size(), false);

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (filtered[i]) continue;
        
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (filtered[j]) continue;
            
            // Check if i would filter j (j behind i's plane)
            glm::dvec3 toJ = candidates[j].position - candidates[i].position;
            double signedDistItoJ = glm::dot(toJ, candidates[i].normal);
            bool iFiltersJ = (signedDistItoJ < planeBias);
            
            // Check if j would filter i (i behind j's plane)
            glm::dvec3 toI = candidates[i].position - candidates[j].position;
            double signedDistJtoI = glm::dot(toI, candidates[j].normal);
            bool jFiltersI = (signedDistJtoI < planeBias);
            
            if (iFiltersJ && jFiltersI) {
                // Mutual filtering - keep the closer one
                if (candidates[i].distance < candidates[j].distance) {
                    filtered[j] = true;
                } else {
                    filtered[i] = true;
                }
            } else if (iFiltersJ) {
                filtered[j] = true;
            } else if (jFiltersI) {
                filtered[i] = true;
            }
        }
    }

    // ========== Step 2.6: Angle-Based Filtering ==========
    // Filter out surfaces that are too steep to be considered ground
    // Use stricter angle when orientation is locked
    glm::dvec3 comparisonDirection = usingCache ? modifiedUp : -downDirection;
    double maxAngle = usingCache ? m_maxLockedGroundAngle : m_maxGroundAngle;
    double angleThreshold = glm::cos(maxAngle);
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (filtered[i]) continue;
        
        const glm::dvec3& normal = candidates[i].normal;
        double alignment = glm::dot(normal, comparisonDirection);
        
        if (alignment < angleThreshold) {
            filtered[i] = true;
        }
    }

    // ========== Step 3: Find Best Ground Contact Point ==========
    bool foundContact = false;
    glm::dvec3 closestPoint(0.0);
    double bestScore = std::numeric_limits<double>::max();
    std::weak_ptr<RigidBody> closestRigidBody;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (filtered[i]) continue;
        
        const glm::dvec3& contactPoint = candidates[i].position;
        const glm::dvec3& normal = candidates[i].normal;
        double distance = candidates[i].distance;
        
        // Calculate score using alignment with modified up direction
        double score;
        double bodyAlignment = glm::dot(normal, modifiedUp);
        score = distance - m_groundSelectionBias * bodyAlignment;
        
        if (score < bestScore) {
            bestScore = score;
            closestPoint = contactPoint;
            foundContact = true;
            closestRigidBody = candidates[i].rigidBody;
        }
    }

    //// ========== DEBUG: GeoGebra Output ==========
    //std::cout << "// GeoGebra 3D Points for contact candidates:" << std::endl;
    //for (size_t i = 0; i < candidates.size(); ++i) {
    //    //if (filtered[i]) continue;
    //    
    //    const glm::dvec3& pos = candidates[i].position;
    //    const glm::dvec3& normal = candidates[i].normal;
    //    
    //    std::cout << "P" << i << " = (" 
    //              << pos.x << ", " 
    //              << pos.y << ", " 
    //              << pos.z << ")" << std::endl;
    //    
    //    glm::dvec3 normalEnd = pos + normal * 0.3; // Scale normal for visibility
    //    std::cout << "N" << i << " = Vector(P" << i << ", (" 
    //              << normalEnd.x << ", " 
    //              << normalEnd.y << ", " 
    //              << normalEnd.z << "))" << std::endl;
    //}
    //if (foundContact) {
    //    std::cout << "ClosestPoint = (" 
    //              << closestPoint.x << ", " 
    //              << closestPoint.y << ", " 
    //              << closestPoint.z << ")" << std::endl;
    //}
    //std::cout << std::endl;
    
    if (!foundContact) {
        m_hasGroundContact = false;
        m_framesWithoutContact++;
        if (m_framesWithoutContact >= m_physicsEngine->getPhysicsHz()) { // 1 second
            m_cachedRigidBody.reset();
            m_cachedModifiedUp = glm::dvec3(0.0, 0.0, 0.0);
            m_framesWithoutContact = 0;
        }
        return;
    }

    m_framesWithoutContact = 0;

    // ========== Step 4: Store Target Rigid Body ==========
    m_walkingTargetRigidBody = closestRigidBody;
    auto targetRigidBody = m_walkingTargetRigidBody.lock();

    // ========== Step 5: Calculate Normal ==========
    glm::dvec3 toBody = rigidBody->m_position - closestPoint;
    double toBodyLengthSq = glm::length2(toBody);
    glm::dvec3 normal;
    if (toBodyLengthSq < 1e-12) {
        // Degenerate case - pick arbitrary up direction
        normal = rigidBody->m_orientation * glm::dvec3(0.0, 0.0, 1.0);
    } else {
        normal = toBody / glm::sqrt(toBodyLengthSq);
    }
    
    // Debug visualization of closest contact point
    if (DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->createSphere(
            "closest_contact", closestPoint, 0.1);
    }

    // ========== Step 6: Calculate Surface Alignment for Force Scaling ==========
    // Calculate alignment between surface normal and locked up direction (for movement force scaling)
    double surfaceAlignment = usingCache ? glm::dot(normal, modifiedUp) : 1.0;

    glm::dvec3 targetUpDirection = usingCache ? modifiedUp : normal;

    m_hasGroundContact    = true;
    m_groundContactPoint  = closestPoint;
    // Surface normal for the animation is always the direction to the body, regardless of
    // whether the orientation up is locked (targetUpDirection). Otherwise the feet would
    // follow the frozen orientation up instead of the surface when the grid rotates.
    m_groundSurfaceNormal = normal;

    // Set orientation cache when establishing new lock (not using cache yet)
    if (!usingCache && targetRigidBody) {
        m_cachedRigidBody = targetRigidBody;
        m_cachedModifiedUp = glm::conjugate(targetRigidBody->m_orientation) * normal;
    }

    // ========== Accumulators for Force/Torque ==========
    glm::dvec3 netForceOnDigibot(0.0);
    glm::dvec3 netTorqueOnDigibot(0.0);

    // ========== Step 7: Unified Orientation Control ==========
    // Project view direction onto tangent plane
    glm::dvec3 projectedViewDirection = m_viewDirection - glm::dot(m_viewDirection, targetUpDirection) * targetUpDirection;
    double projectedViewLengthSq = glm::length2(projectedViewDirection);
    
    glm::dvec3 targetForward;
    if (projectedViewLengthSq < 1e-12) {
        // View is aligned with normal - keep current forward direction
        targetForward = rigidBody->m_orientation * glm::dvec3(0.0, 1.0, 0.0);
        targetForward = targetForward - glm::dot(targetForward, targetUpDirection) * targetUpDirection;
        double targetForwardLengthSq = glm::length2(targetForward);
        if (targetForwardLengthSq < 1e-12) {
            // Current forward also aligned with normal - pick arbitrary tangent
            glm::dvec3 arbitrary = glm::dvec3(1.0, 0.0, 0.0);
            if (glm::abs(glm::dot(targetUpDirection, arbitrary)) > 0.9) {
                arbitrary = glm::dvec3(0.0, 1.0, 0.0);
            }
            targetForward = arbitrary - glm::dot(arbitrary, targetUpDirection) * targetUpDirection;
            targetForward = glm::normalize(targetForward);
        } else {
            targetForward = targetForward / glm::sqrt(targetForwardLengthSq);
        }
    } else {
        targetForward = projectedViewDirection / glm::sqrt(projectedViewLengthSq);    
    }

    // Construct target orientation from orthonormal basis
    glm::dvec3 targetRight = glm::cross(targetForward, targetUpDirection);
    glm::dvec3 targetUp = targetUpDirection;
    
    // Build rotation matrix and convert to quaternion
    glm::dmat3 targetRotationMatrix;
    targetRotationMatrix[0] = targetRight;   // x-axis
    targetRotationMatrix[1] = targetForward; // y-axis
    targetRotationMatrix[2] = targetUp;      // z-axis
    glm::dquat targetOrientation = glm::quat_cast(targetRotationMatrix);

    // Ensure we take the short path: negate target if dot product is negative
    // (q and -q represent the same rotation, so pick the closer one)
    if (glm::dot(targetOrientation, rigidBody->m_orientation) < 0.0) {
        targetOrientation = -targetOrientation;
    }
    
    // Calculate rotation from current to target orientation
    glm::dquat rotationDelta = targetOrientation * glm::conjugate(rigidBody->m_orientation);
    //rotationDelta = glm::normalize(rotationDelta);
    
    // Extract axis-angle from rotation delta
    double deltaAngle = 2.0 * std::acos(glm::clamp(rotationDelta.w, -1.0, 1.0));
    glm::dvec3 rotationAxis(rotationDelta.x, rotationDelta.y, rotationDelta.z);
    double axisLength = glm::length(rotationAxis);
    
    glm::dvec3 targetAngularVelocity(0.0);
    
    // Scale acceleration based on remaining angle
    double effectiveAngAcc = m_angularAccelerationMax;
    effectiveAngAcc *= glm::min(glm::abs(deltaAngle) / 0.1, 1.0);
    
    if (axisLength > 1e-6 && deltaAngle > 1e-6) {
        rotationAxis = rotationAxis / axisLength;
        
        // Calculate target angular speed using sqrt(2ad) with margin
        double margin = 0.5;
        double maxAngularSpeed = std::sqrt(2.0 * effectiveAngAcc * (1.0 - margin) * deltaAngle);
        targetAngularVelocity = rotationAxis * maxAngularSpeed;
    }
    
    // Add grid's angular velocity if walking on moving surface
    if (targetRigidBody) {
        targetAngularVelocity += targetRigidBody->getAngularVelocityWorld();
    }
    
    // Calculate needed angular acceleration
    glm::dvec3 currentAngularVelocity = rigidBody->getAngularVelocityWorld();
    glm::dvec3 angularAcceleration = targetAngularVelocity - currentAngularVelocity;
    
    // Limit angular acceleration
    double angAccMagnitude = glm::length(angularAcceleration);
    if (angAccMagnitude > effectiveAngAcc) {
        angularAcceleration = angularAcceleration * (effectiveAngAcc / angAccMagnitude);
    }
        
    // Calculate torque using effective inertia
    glm::dvec3 orientationTorque;
    if (targetRigidBody && !targetRigidBody->m_isStatic) {
        // Calculate effective inertia between Digibot and moving ground
        glm::dmat3 invInertiaSum = rigidBody->getWorldInvInertiaTensor() + 
                                    targetRigidBody->getWorldInvInertiaTensor();
        glm::dmat3 effectiveInertia = glm::inverse(invInertiaSum);
        orientationTorque = effectiveInertia * angularAcceleration;
    } else {
        // Static ground or no ground - use only Digibot's inertia
        orientationTorque = rigidBody->getWorldInertiaTensor() * angularAcceleration;
    }
    
    netTorqueOnDigibot += orientationTorque;
    
    // ========== Step 8: Position Control Along Normal ==========
    // Calculate target position along target up direction
    glm::dvec3 targetPosition = closestPoint + targetUpDirection * m_targetHoverHeight;
    
    // Calculate position error along target up direction only
    glm::dvec3 positionError = targetPosition - rigidBody->m_position;
    double distanceAlongNormal = glm::dot(positionError, targetUpDirection);
    
    // Calculate target speed along target up direction using sqrt(2ad)
    double margin = 0.5;
    double effectiveACC = m_maxGroundAcceleration;
    effectiveACC *= glm::min(glm::abs(distanceAlongNormal) / 0.1, 1.);
    double targetSpeedAlongNormal = std::sqrt(2.0 * effectiveACC * (1.0 - margin) * glm::abs(distanceAlongNormal));
    if (distanceAlongNormal < 0.0) {
        targetSpeedAlongNormal = -targetSpeedAlongNormal;
    }
    
    // Calculate surface velocity at contact point
    double surfaceVelocityAlongNormal = 0.0;
    if (targetRigidBody) {
        // Linear velocity of surface
        glm::dvec3 surfaceLinearVelocity = targetRigidBody->m_velocity;
        
        // Velocity from rotation: v = ω × r
        glm::dvec3 radiusVector = closestPoint - targetRigidBody->m_position;
        glm::dvec3 velocityFromRotation = glm::cross(targetRigidBody->getAngularVelocityWorld(), radiusVector);
        
        glm::dvec3 surfaceVelocityAtContact = surfaceLinearVelocity + velocityFromRotation;
        surfaceVelocityAlongNormal = glm::dot(surfaceVelocityAtContact, targetUpDirection);
    }
    
    // Add surface velocity to target speed
    double targetVelocityAlongNormal = targetSpeedAlongNormal + surfaceVelocityAlongNormal;
    
    // Calculate needed acceleration along target up direction
    double currentVelocityAlongNormal = glm::dot(rigidBody->m_velocity, targetUpDirection);
    double accelerationAlongNormal = targetVelocityAlongNormal - currentVelocityAlongNormal;
    
    // Clamp acceleration
    if (glm::abs(accelerationAlongNormal) > effectiveACC) {
        accelerationAlongNormal = (accelerationAlongNormal > 0.0 ? 1.0 : -1.0) * effectiveACC;
    }
     
    glm::dvec3 neededAcceleration = targetUpDirection * accelerationAlongNormal;
    
    // Apply force to character
    // Calculate effective mass for hover control
    double effectiveMass;
    if (targetRigidBody) {
        // Calculate effective mass between Digibot and ground
        //glm::dvec3 rDigibot = glm::dvec3(0.0); // Force at COM, so r = 0
        glm::dvec3 rGround = rigidBody->m_position - targetRigidBody->m_position;
        
        glm::dvec3 rGround_cross_n = glm::cross(rGround, targetUpDirection);
        glm::dvec3 rotContribGround = targetRigidBody->getWorldInvInertiaTensor() * rGround_cross_n;
        double rotTermGround = glm::dot(rGround_cross_n, rotContribGround);
        
        double invEffectiveMass = rigidBody->m_invMass + targetRigidBody->m_invMass + rotTermGround;
        effectiveMass = 1.0 / invEffectiveMass;
    } else {
        effectiveMass = rigidBody->m_mass;
    }
    
    glm::dvec3 hoverForce = neededAcceleration * effectiveMass;

    netForceOnDigibot += hoverForce;

    // ========== Step 9: Apply Movement Force ==========
    // Create 2D tangent space using targetUpDirection
    // This uses locked up when cached, or surface normal when unlocked
    glm::dvec3 tangentX = glm::cross(m_viewDirection, targetUpDirection);
    double tangentXLengthSq = glm::length2(tangentX);
    
    // If view direction is too aligned with up direction, use a fallback
    if (tangentXLengthSq < 1e-12) {
        // View is aligned with up direction, pick arbitrary tangent direction
        glm::dvec3 arbitrary = glm::dvec3(1.0, 0.0, 0.0);
        if (glm::abs(glm::dot(targetUpDirection, arbitrary)) > 0.9) {
            arbitrary = glm::dvec3(0.0, 1.0, 0.0);
        }
        tangentX = glm::cross(arbitrary, targetUpDirection);
        tangentXLengthSq = glm::length2(tangentX);
    }
    
    tangentX = tangentX / glm::sqrt(tangentXLengthSq);
    glm::dvec3 tangentY = glm::cross(targetUpDirection, tangentX);
    
    // Calculate surface velocity at body position
    glm::dvec3 surfaceVelocityAtBody(0.0);
    if (targetRigidBody) {
        glm::dvec3 radiusVector = rigidBody->m_position - targetRigidBody->m_position;
        glm::dvec3 velocityFromRotation = glm::cross(targetRigidBody->getAngularVelocityWorld(), radiusVector);
        surfaceVelocityAtBody = targetRigidBody->m_velocity + velocityFromRotation;
    }
    
    // Calculate relative velocity in tangent plane (remove normal component)
    glm::dvec3 relativeVelocity3D = rigidBody->m_velocity - surfaceVelocityAtBody;
    glm::dvec3 relativeVelocityTangent = relativeVelocity3D - glm::dot(relativeVelocity3D, normal) * normal;
    
    // Create target velocity from movement input in world space
    glm::dvec3 targetVelocityDirection = 
        static_cast<double>(m_movementDirection.x) * tangentX + 
        static_cast<double>(m_movementDirection.y) * tangentY;
    
    // Scale by target walk speed
    double targetVelocityMagnitude = glm::length(targetVelocityDirection);
    if (targetVelocityMagnitude > 1e-6) {
        targetVelocityDirection = (targetVelocityDirection / targetVelocityMagnitude) * m_targetWalkSpeed;
    }

    // Project target velocity onto actual surface plane to ensure it's achievable
    targetVelocityDirection = targetVelocityDirection - glm::dot(targetVelocityDirection, normal) * normal;
    
    glm::dvec3 velocityError = targetVelocityDirection - relativeVelocityTangent;
    // Calculate effective mass for movement control in the direction of velocity error
    double effectiveMassForMovement;
    if (targetRigidBody && glm::length(velocityError) > 1e-6) {
        glm::dvec3 movementDirection = glm::normalize(velocityError);
        
        // Force at Digibot COM, so no rotational contribution for Digibot
        glm::dvec3 rGround = rigidBody->m_position - targetRigidBody->m_position;
        
        glm::dvec3 rGround_cross_dir = glm::cross(rGround, movementDirection);
        glm::dvec3 rotContribGround = targetRigidBody->getWorldInvInertiaTensor() * rGround_cross_dir;
        double rotTermGround = glm::dot(rGround_cross_dir, rotContribGround);
        
        double invEffectiveMass = rigidBody->m_invMass + targetRigidBody->m_invMass + rotTermGround;
        effectiveMassForMovement = 1.0 / invEffectiveMass;
    } else {
        effectiveMassForMovement = rigidBody->m_mass;
    }
    
    double forceMagnitude = m_walkingThrustStrength * effectiveMassForMovement;
    
    //// Double force when trying to change direction (opposite velocities)
    //if (glm::dot(targetVelocityDirection, relativeVelocityTangent) < 0.0) {
    //    forceMagnitude *= 2.0;
    //}

    double smoothFactor = 0.5;
    glm::dvec3 movementForce = velocityError * effectiveMassForMovement *
        smoothFactor;
    double movementForceLength = glm::length(movementForce);
    if (movementForceLength > forceMagnitude)
    {
        movementForce = movementForce / movementForceLength * forceMagnitude;
    }
    
    // Scale movement force by alignment when using cached direction
    if (usingCache) {
        double scaleFactor = glm::max(0.0, surfaceAlignment);
        movementForce *= scaleFactor;
    }
    
    if (glm::length(movementForce) > 1e-6) {
        netForceOnDigibot += movementForce;
    }

    // ========== Apply All Accumulated Forces/Torques ==========
    m_physicsEngine->applyForce(rigidBodyWeak, netForceOnDigibot);
    m_physicsEngine->applyTorque(rigidBodyWeak, netTorqueOnDigibot);

    // Apply equal and opposite reactions to grid at contact point
    if (targetRigidBody) {
        m_physicsEngine->applyForceAtPoint(m_walkingTargetRigidBody, -netForceOnDigibot, rigidBody->m_position);
        m_physicsEngine->applyTorque(m_walkingTargetRigidBody, -netTorqueOnDigibot);
    }
}

void DigibotController::setThrustStrength(double strength) {
    m_thrustStrength = strength;
}

void DigibotController::setRollAcceleration(double acceleration) {
    m_rollAcceleration = acceleration;
}