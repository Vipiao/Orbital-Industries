// DigibotWalkingMode.cpp
#include "DigibotWalkingMode.h"
#include "DigibotPhysics.h"
#include "../MotionServo.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/RigidBody.h"
#include "../../physics/RotatingFrameUtils.h"
#include "../../physics/SensorCollider.h"
#include "debug/DebugGlobals.h"
#include "debug/DebugRenderer.h"
#include <glm/gtx/norm.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

DigibotWalkingMode::DigibotWalkingMode(DigibotPhysics* physics,
                                       PhysicsEngine* physicsEngine)
    : m_physics{physics}
    , m_physicsEngine{physicsEngine}
{
    if (!m_physics) {
        throw std::runtime_error("DigibotWalkingMode: Physics component cannot be null");
    }
    if (!m_physicsEngine) {
        throw std::runtime_error("DigibotWalkingMode: Physics engine cannot be null");
    }
}

void DigibotWalkingMode::resetContactState() {
    m_cachedRigidBody.reset();
    m_cachedModifiedUp = glm::dvec3{0.0, 0.0, 0.0};
    m_framesWithoutContact = 0;
    m_hasGroundContact = false;
    m_walkingTargetRigidBody.reset();
}

DigibotWrench DigibotWalkingMode::update(const std::shared_ptr<RigidBody>& rigidBody,
                                         const DigibotModeInputs& inputs) {
    DigibotWrench wrench{};

    // DEBUG
    if (DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("closest_contact");
    }

    // Clear walking target at start (will be set if we find ground contact)
    m_walkingTargetRigidBody.reset();

    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return wrench;
    }

    // Calculate reference down direction from body orientation
    glm::dvec3 downDirection{-(rigidBody->m_orientation * glm::dvec3{0.0, 0.0, 1.0})};

    // Handle cached modified up direction
    glm::dvec3 modifiedUp{0.0, 0.0, 0.0};
    bool usingCache{false};

    if (!inputs.m_isUpDirectionLocked) {
        // Direction is free to change - clear cache
        m_cachedRigidBody.reset();
    } else {
        // Direction is locked - try to use cache
        if (!m_cachedRigidBody.expired()) {
            auto cachedRigidBody = m_cachedRigidBody.lock();
            // Verify cached rigid body still exists
            bool isValid{false};
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
        glm::dvec3 bodyUpDirection{rigidBody->m_orientation * glm::dvec3{0.0, 0.0, 1.0}};
        glm::dvec3 robotRight{rigidBody->m_orientation * glm::dvec3{1.0, 0.0, 0.0}};
        modifiedUp = glm::cross(robotRight, inputs.m_viewDirection);
        double modifiedUpLengthSq{glm::length2(modifiedUp)};
        if (modifiedUpLengthSq < 1e-12) {
            // View direction parallel to robot right - fall back to body up
            modifiedUp = bodyUpDirection;
        } else {
            modifiedUp = modifiedUp / glm::sqrt(modifiedUpLengthSq);
        }
    }

    auto sensor = m_physics->getWalkingSensor().lock();
    if (!sensor) {
        return wrench;
    }

    const auto& collisions = m_physicsEngine->getCollisions(sensor.get());

    // ========== Step 1: Collect All Contact Candidates ==========
    struct ContactCandidate {
        glm::dvec3 position;
        glm::dvec3 normal;
        double distance;
        std::weak_ptr<RigidBody> rigidBody;
    };

    std::vector<ContactCandidate> candidates{};

    for (const auto& collision : collisions) {
        // Skip collisions with the robot's own body
        ColliderAttachment* otherAttachment{
            collision.otherCollider->get_pointer<ColliderAttachment>()};
        std::weak_ptr<RigidBody> otherRigidBody{};
        if (otherAttachment) {
            auto otherRigidBodyShared = otherAttachment->rigidBody.lock();
            if (otherRigidBodyShared && otherRigidBodyShared.get() == rigidBody.get()) {
                continue;
            }
            otherRigidBody = otherAttachment->rigidBody;
        }
        for (const auto& contactPoint : collision.contactPoints) {
            // Calculate surface normal (points from surface toward body)
            glm::dvec3 toBody{rigidBody->m_position - contactPoint};
            double distance{glm::length(toBody)};

            if (distance < 1e-6) {
                continue; // Skip degenerate case
            }

            glm::dvec3 normal{toBody / distance};
            candidates.push_back({contactPoint, normal, distance, otherRigidBody});
        }
    }

    // ========== Step 2: Plane-Based Filtering ==========
    // Filter out points that are recessed relative to other points
    const double planeBias{0.01};
    std::vector<bool> filtered(candidates.size(), false);

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (filtered[i]) continue;

        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (filtered[j]) continue;

            // Check if i would filter j (j behind i's plane)
            glm::dvec3 toJ{candidates[j].position - candidates[i].position};
            double signedDistItoJ{glm::dot(toJ, candidates[i].normal)};
            bool iFiltersJ{signedDistItoJ < planeBias};

            // Check if j would filter i (i behind j's plane)
            glm::dvec3 toI{candidates[i].position - candidates[j].position};
            double signedDistJtoI{glm::dot(toI, candidates[j].normal)};
            bool jFiltersI{signedDistJtoI < planeBias};

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
    glm::dvec3 comparisonDirection{usingCache ? modifiedUp : -downDirection};
    double maxAngle{usingCache ? m_maxLockedGroundAngle : m_maxGroundAngle};
    double angleThreshold{glm::cos(maxAngle)};

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (filtered[i]) continue;

        double alignment{glm::dot(candidates[i].normal, comparisonDirection)};
        if (alignment < angleThreshold) {
            filtered[i] = true;
        }
    }

    // ========== Step 3: Find Best Ground Contact Point ==========
    bool foundContact{false};
    glm::dvec3 closestPoint{0.0};
    double bestScore{std::numeric_limits<double>::max()};
    std::weak_ptr<RigidBody> closestRigidBody{};

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (filtered[i]) continue;

        // Calculate score using alignment with modified up direction
        double bodyAlignment{glm::dot(candidates[i].normal, modifiedUp)};
        double score{candidates[i].distance - m_groundSelectionBias * bodyAlignment};

        if (score < bestScore) {
            bestScore = score;
            closestPoint = candidates[i].position;
            foundContact = true;
            closestRigidBody = candidates[i].rigidBody;
        }
    }

    if (!foundContact) {
        m_hasGroundContact = false;
        m_framesWithoutContact++;
        if (m_framesWithoutContact >= m_physicsEngine->getPhysicsHz()) { // 1 second
            m_cachedRigidBody.reset();
            m_cachedModifiedUp = glm::dvec3{0.0, 0.0, 0.0};
            m_framesWithoutContact = 0;
        }
        return wrench;
    }

    m_framesWithoutContact = 0;

    // ========== Step 4: Store Target Rigid Body ==========
    m_walkingTargetRigidBody = closestRigidBody;
    auto targetRigidBody = m_walkingTargetRigidBody.lock();

    // ========== Step 5: Calculate Normal ==========
    glm::dvec3 toBody{rigidBody->m_position - closestPoint};
    double toBodyLengthSq{glm::length2(toBody)};
    glm::dvec3 normal{};
    if (toBodyLengthSq < 1e-12) {
        // Degenerate case - pick arbitrary up direction
        normal = rigidBody->m_orientation * glm::dvec3{0.0, 0.0, 1.0};
    } else {
        normal = toBody / glm::sqrt(toBodyLengthSq);
    }

    // Debug visualization of closest contact point
    if (DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->createSphere(
            "closest_contact", closestPoint, 0.1);
    }

    // ========== Step 6: Calculate Surface Alignment for Force Scaling ==========
    double surfaceAlignment{usingCache ? glm::dot(normal, modifiedUp) : 1.0};

    glm::dvec3 targetUpDirection{usingCache ? modifiedUp : normal};

    m_hasGroundContact = true;
    m_groundContactPoint = closestPoint;
    // Surface normal for the animation is always the direction to the body, regardless
    // of whether the orientation up is locked (targetUpDirection). Otherwise the feet
    // would follow the frozen orientation up instead of the surface when the grid
    // rotates.
    m_groundSurfaceNormal = normal;

    // Set orientation cache when establishing new lock (not using cache yet)
    if (!usingCache && targetRigidBody) {
        m_cachedRigidBody = targetRigidBody;
        m_cachedModifiedUp = glm::conjugate(targetRigidBody->m_orientation) * normal;
    }

    // ========== Accumulators for Force/Torque ==========
    glm::dvec3 netForceOnDigibot{0.0};
    glm::dvec3 netTorqueOnDigibot{0.0};

    // ========== Step 7: Unified Orientation Control ==========
    // Project view direction onto tangent plane
    glm::dvec3 projectedViewDirection{
        inputs.m_viewDirection -
        glm::dot(inputs.m_viewDirection, targetUpDirection) * targetUpDirection};
    double projectedViewLengthSq{glm::length2(projectedViewDirection)};

    glm::dvec3 targetForward{};
    if (projectedViewLengthSq < 1e-12) {
        // View is aligned with normal - keep current forward direction
        targetForward = rigidBody->m_orientation * glm::dvec3{0.0, 1.0, 0.0};
        targetForward = targetForward -
                        glm::dot(targetForward, targetUpDirection) * targetUpDirection;
        double targetForwardLengthSq{glm::length2(targetForward)};
        if (targetForwardLengthSq < 1e-12) {
            // Current forward also aligned with normal - pick arbitrary tangent
            glm::dvec3 arbitrary{1.0, 0.0, 0.0};
            if (glm::abs(glm::dot(targetUpDirection, arbitrary)) > 0.9) {
                arbitrary = glm::dvec3{0.0, 1.0, 0.0};
            }
            targetForward = arbitrary -
                            glm::dot(arbitrary, targetUpDirection) * targetUpDirection;
            targetForward = glm::normalize(targetForward);
        } else {
            targetForward = targetForward / glm::sqrt(targetForwardLengthSq);
        }
    } else {
        targetForward = projectedViewDirection / glm::sqrt(projectedViewLengthSq);
    }

    // Construct target orientation from orthonormal basis
    glm::dvec3 targetRight{glm::cross(targetForward, targetUpDirection)};

    glm::dmat3 targetRotationMatrix{};
    targetRotationMatrix[0] = targetRight;       // x-axis
    targetRotationMatrix[1] = targetForward;     // y-axis
    targetRotationMatrix[2] = targetUpDirection; // z-axis
    glm::dquat targetOrientation{glm::quat_cast(targetRotationMatrix)};

    MotionServo::AngularTarget angularTarget{MotionServo::towardOrientation(
        rigidBody->m_orientation, targetOrientation, m_angularAccelerationMax, 0.5,
        0.1)};

    // Add grid's angular velocity if walking on moving surface
    glm::dvec3 extraAngularVelocity{0.0, 0.0, 0.0};
    if (targetRigidBody) {
        extraAngularVelocity = targetRigidBody->getAngularVelocityWorld();
    }

    glm::dmat3 orientationInertia{
        RotatingFrameUtils::effectiveInertia(*rigidBody, targetRigidBody.get())};
    netTorqueOnDigibot += MotionServo::torque(
        angularTarget, extraAngularVelocity, rigidBody->getAngularVelocityWorld(),
        orientationInertia);

    // ========== Step 8: Position Control Along Normal ==========
    // Calculate target position along target up direction
    glm::dvec3 targetPosition{closestPoint + targetUpDirection * m_targetHoverHeight};

    // Calculate position error along target up direction only
    glm::dvec3 positionError{targetPosition - rigidBody->m_position};
    double distanceAlongNormal{glm::dot(positionError, targetUpDirection)};

    // Velocity relative to the surface at the contact point, along the up direction
    double surfaceVelocityAlongNormal{0.0};
    if (targetRigidBody) {
        glm::dvec3 surfaceVelocityAtContact{
            RotatingFrameUtils::velocityAtPoint(*targetRigidBody, closestPoint)};
        surfaceVelocityAlongNormal = glm::dot(surfaceVelocityAtContact, targetUpDirection);
    }
    double relativeVelocityAlongNormal{
        glm::dot(rigidBody->m_velocity, targetUpDirection) - surfaceVelocityAlongNormal};

    // sqrt(2ad) approach along the up axis. Acceleration ramps down near the target
    // (prevents overshoot chatter at rest); margin keeps the profile decelerable.
    double margin{0.5};
    double effectiveACC{m_maxGroundAcceleration *
                        glm::min(glm::abs(distanceAlongNormal) / 0.1, 1.0)};
    glm::dvec3 targetVelocityAlongNormal{MotionServo::velocityToward(
        targetUpDirection * distanceAlongNormal, effectiveACC * (1.0 - margin))};

    glm::dvec3 neededAcceleration{
        targetVelocityAlongNormal - targetUpDirection * relativeVelocityAlongNormal};
    double neededMagnitude{glm::length(neededAcceleration)};
    if (neededMagnitude > effectiveACC) {
        neededAcceleration = neededAcceleration * (effectiveACC / neededMagnitude);
    }

    // Effective mass for hover control (force at digibot COM, reaction on ground)
    glm::dvec3 reactionOffset{0.0, 0.0, 0.0};
    if (targetRigidBody) {
        reactionOffset = rigidBody->m_position - targetRigidBody->m_position;
    }
    double effectiveMass{RotatingFrameUtils::effectiveMass(
        *rigidBody, targetRigidBody.get(), targetUpDirection, reactionOffset)};

    netForceOnDigibot += neededAcceleration * effectiveMass;

    // ========== Step 9: Apply Movement Force ==========
    // Create 2D tangent space using targetUpDirection
    glm::dvec3 tangentX{glm::cross(inputs.m_viewDirection, targetUpDirection)};
    double tangentXLengthSq{glm::length2(tangentX)};

    // If view direction is too aligned with up direction, use a fallback
    if (tangentXLengthSq < 1e-12) {
        glm::dvec3 arbitrary{1.0, 0.0, 0.0};
        if (glm::abs(glm::dot(targetUpDirection, arbitrary)) > 0.9) {
            arbitrary = glm::dvec3{0.0, 1.0, 0.0};
        }
        tangentX = glm::cross(arbitrary, targetUpDirection);
        tangentXLengthSq = glm::length2(tangentX);
    }

    tangentX = tangentX / glm::sqrt(tangentXLengthSq);
    glm::dvec3 tangentY{glm::cross(targetUpDirection, tangentX)};

    // Calculate surface velocity at body position
    glm::dvec3 surfaceVelocityAtBody{0.0, 0.0, 0.0};
    if (targetRigidBody) {
        surfaceVelocityAtBody =
            RotatingFrameUtils::velocityAtPoint(*targetRigidBody, rigidBody->m_position);
    }

    // Calculate relative velocity in tangent plane (remove normal component)
    glm::dvec3 relativeVelocity3D{rigidBody->m_velocity - surfaceVelocityAtBody};
    glm::dvec3 relativeVelocityTangent{
        relativeVelocity3D - glm::dot(relativeVelocity3D, normal) * normal};

    // Create target velocity from movement input in world space
    glm::dvec3 targetVelocityDirection{
        static_cast<double>(inputs.m_movementDirection.x) * tangentX +
        static_cast<double>(inputs.m_movementDirection.y) * tangentY};

    // Scale by target walk speed
    double targetVelocityMagnitude{glm::length(targetVelocityDirection)};
    if (targetVelocityMagnitude > 1e-6) {
        targetVelocityDirection =
            (targetVelocityDirection / targetVelocityMagnitude) * m_targetWalkSpeed;
    }

    // Project target velocity onto actual surface plane to ensure it's achievable
    targetVelocityDirection =
        targetVelocityDirection - glm::dot(targetVelocityDirection, normal) * normal;

    glm::dvec3 velocityError{targetVelocityDirection - relativeVelocityTangent};

    // Effective mass for movement control in the direction of the velocity error
    double effectiveMassForMovement{rigidBody->m_mass};
    if (targetRigidBody && glm::length(velocityError) > 1e-6) {
        glm::dvec3 movementDirection{glm::normalize(velocityError)};
        effectiveMassForMovement = RotatingFrameUtils::effectiveMass(
            *rigidBody, targetRigidBody.get(), movementDirection, reactionOffset);
    }

    double forceMagnitude{m_walkingThrustStrength * effectiveMassForMovement};

    double smoothFactor{0.5};
    glm::dvec3 movementForce{velocityError * effectiveMassForMovement * smoothFactor};
    double movementForceLength{glm::length(movementForce)};
    if (movementForceLength > forceMagnitude) {
        movementForce = movementForce / movementForceLength * forceMagnitude;
    }

    // Scale movement force by alignment when using cached direction
    if (usingCache) {
        movementForce *= glm::max(0.0, surfaceAlignment);
    }

    if (glm::length(movementForce) > 1e-6) {
        netForceOnDigibot += movementForce;
    }

    // ========== Return Accumulated Wrench ==========
    wrench.m_force = netForceOnDigibot;
    wrench.m_torque = netTorqueOnDigibot;
    wrench.m_reactionBody = m_walkingTargetRigidBody;
    return wrench;
}
