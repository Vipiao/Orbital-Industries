// DigibotWalkingMode.cpp
#include "DigibotWalkingMode.h"
#include "DigibotPhysics.h"
#include "../MotionServo.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/RigidBody.h"
#include "../../physics/RotatingFrameUtils.h"
#include "../../physics/SensorCollider.h"
#include <cassert>
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

glm::dvec3 DigibotWalkingMode::getGroundContactPoint() const {
    std::shared_ptr<RigidBody> body{m_walkingTargetRigidBody.lock()};
    if (!body) {
        return m_groundContactPointLocal;
    }
    return body->getPosition() + body->getOrientation() * m_groundContactPointLocal;
}

glm::dvec3 DigibotWalkingMode::getGroundSurfaceNormal() const {
    std::shared_ptr<RigidBody> body{m_walkingTargetRigidBody.lock()};
    if (!body) {
        return m_groundSurfaceNormalLocal;
    }
    return body->getOrientation() * m_groundSurfaceNormalLocal;
}

DigibotWrench DigibotWalkingMode::update(const std::shared_ptr<RigidBody>& rigidBody,
                                         const DigibotModeInputs& inputs) {
    DigibotWrench wrench{};

    // Clear walking target at start (will be set if we find ground contact)
    m_walkingTargetRigidBody.reset();

    if (!rigidBody || rigidBody->getMass() <= 0.0) {
        return wrench;
    }

    // Calculate reference down direction from body orientation
    glm::dvec3 downDirection{-(rigidBody->getOrientation() * glm::dvec3{0.0, 0.0, 1.0})};

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
                modifiedUp = cachedRigidBody->getOrientation() * m_cachedModifiedUp;
                usingCache = true;
            } else {
                // Cached rigid body was destroyed - clear cache and recalculate
                m_cachedRigidBody.reset();
            }
        }
    }

    // Calculate fresh modifiedUp if needed
    if (!usingCache) {
        glm::dvec3 bodyUpDirection{rigidBody->getOrientation() * glm::dvec3{0.0, 0.0, 1.0}};
        glm::dvec3 robotRight{rigidBody->getOrientation() * glm::dvec3{1.0, 0.0, 0.0}};
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
            // Trigger volumes (e.g. another character's walking sensor) are not
            // walkable ground.
            if (otherAttachment->isTrigger) {
                continue;
            }
            otherRigidBody = otherAttachment->rigidBody;
        }
        assert(collision.contactPoints.size() == collision.contactData.size());
        for (size_t k = 0; k < collision.contactPoints.size(); ++k) {
            // The cached world-space contact is one step stale; rebuilding it at the
            // other collider's current pose keeps it valid across position rewrites
            // (e.g. a network state apply).
            glm::dvec3 contactPoint{
                collision.otherCollider->localToWorld(collision.contactPointsLocalB[k])};
            double distance{glm::length(rigidBody->getPosition() - contactPoint)};

            if (distance < 1e-6) {
                continue; // Skip degenerate case
            }

            // Geometric surface normal from the collision pass, oriented toward the
            // body. A property of the surface, not of the body's live position.
            glm::dvec3 normal{collision.contactData[k].normal};
            if (glm::dot(normal, rigidBody->getPosition() - contactPoint) < 0.0) {
                normal = -normal;
            }
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
    glm::dvec3 closestNormal{0.0};
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
            closestNormal = candidates[i].normal;
            foundContact = true;
            closestRigidBody = candidates[i].rigidBody;
        }
    }

    if (!foundContact) {
        m_hasGroundContact = false;
        m_framesWithoutContact++;
        if (m_framesWithoutContact >= PhysicsUnits::seconds(1.0)) {
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

    // ========== Step 5: Surface Normal ==========
    glm::dvec3 normal{closestNormal};

    // ========== Step 6: Calculate Surface Alignment for Force Scaling ==========
    double surfaceAlignment{usingCache ? glm::dot(normal, modifiedUp) : 1.0};

    glm::dvec3 targetUpDirection{usingCache ? modifiedUp : normal};

    m_hasGroundContact = true;
    // Surface normal for the animation is always the direction to the body, regardless
    // of whether the orientation up is locked (targetUpDirection). Otherwise the feet
    // would follow the frozen orientation up instead of the surface when the grid
    // rotates.
    if (targetRigidBody) {
        glm::dquat toLocal{glm::conjugate(targetRigidBody->getOrientation())};
        m_groundContactPointLocal = toLocal * (closestPoint - targetRigidBody->getPosition());
        m_groundSurfaceNormalLocal = toLocal * normal;
    } else {
        m_groundContactPointLocal = closestPoint;
        m_groundSurfaceNormalLocal = normal;
    }

    // Set orientation cache when establishing new lock (not using cache yet)
    if (!usingCache && targetRigidBody) {
        m_cachedRigidBody = targetRigidBody;
        m_cachedModifiedUp = glm::conjugate(targetRigidBody->getOrientation()) * normal;
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
        targetForward = rigidBody->getOrientation() * glm::dvec3{0.0, 1.0, 0.0};
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

    glm::dvec3 targetAngularVelocity{MotionServo::towardOrientation(
        rigidBody->getOrientation(), targetOrientation, m_angularAccelerationMax, 0.5,
        0.1)};

    // Add grid's angular velocity if walking on moving surface
    glm::dvec3 extraAngularVelocity{0.0, 0.0, 0.0};
    if (targetRigidBody) {
        extraAngularVelocity = targetRigidBody->getAngularVelocityWorld();
    }

    glm::dmat3 orientationInertia{
        RotatingFrameUtils::effectiveInertia(*rigidBody, targetRigidBody.get())};
    netTorqueOnDigibot += MotionServo::torque(
        targetAngularVelocity, extraAngularVelocity, rigidBody->getAngularVelocityWorld(),
        m_angularAccelerationMax, orientationInertia);

    // ========== Step 8: Position Control Along Normal ==========
    // Calculate target position along target up direction
    glm::dvec3 targetPosition{closestPoint + targetUpDirection * m_targetHoverHeight};

    // Calculate position error along target up direction only
    glm::dvec3 positionError{targetPosition - rigidBody->getPosition()};
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

    // sqrt(2ad) approach along the up axis. The near-target ramp tames the sqrt curve
    // at d=0 to stop rest chatter, so it only shapes the profile speed - the correction
    // below keeps full braking authority (m_maxGroundAcceleration), otherwise it cannot
    // arrest residual velocity near the hover height and oscillates.
    double margin{0.5};
    double profileAcceleration{m_maxGroundAcceleration *
                               glm::min(glm::abs(distanceAlongNormal) / 0.1, 1.0)};
    glm::dvec3 targetVelocityAlongNormal{MotionServo::velocityToward(
        targetUpDirection * distanceAlongNormal, profileAcceleration * (1.0 - margin))};

    glm::dvec3 neededAcceleration{
        targetVelocityAlongNormal - targetUpDirection * relativeVelocityAlongNormal};
    double neededMagnitude{glm::length(neededAcceleration)};
    if (neededMagnitude > m_maxGroundAcceleration) {
        neededAcceleration = neededAcceleration * (m_maxGroundAcceleration / neededMagnitude);
    }

    // Effective mass for hover control (force at digibot COM, reaction on ground)
    glm::dvec3 reactionOffset{0.0, 0.0, 0.0};
    if (targetRigidBody) {
        reactionOffset = rigidBody->getPosition() - targetRigidBody->getWorldCenterOfMass();
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
            RotatingFrameUtils::velocityAtPoint(*targetRigidBody, rigidBody->getPosition());
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
    double effectiveMassForMovement{rigidBody->getMass()};
    if (targetRigidBody && glm::length(velocityError) > 1e-6) {
        glm::dvec3 movementDirection{glm::normalize(velocityError)};
        effectiveMassForMovement = RotatingFrameUtils::effectiveMass(
            *rigidBody, targetRigidBody.get(), movementDirection, reactionOffset);
    }

    double forceMagnitude{m_walkingThrustStrength * effectiveMassForMovement};

    // Velocity-tracking gain: multiplies a velocity error to make an acceleration, so
    // it carries 1/time units and must scale with the tick rate (not a bare factor).
    double smoothFactor{PhysicsUnits::perSecond(32.0)};
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
