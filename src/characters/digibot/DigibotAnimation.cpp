// DigibotAnimation.cpp
#include "DigibotAnimation.h"
#include "../../physics/RigidBody.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

namespace {
bool isFinite(const glm::dvec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool isUnit(const glm::dvec3& v) {
    return std::abs(glm::length(v) - 1.0) < 1e-3;
}
}  // namespace

// Skeleton constants match DigibotGraphics natural positions exactly
const glm::dvec3 DigibotAnimation::s_naturalRightHipLocal  = glm::dvec3( 0.177658,  0.061087, 1.02951);
const glm::dvec3 DigibotAnimation::s_naturalLeftHipLocal   = glm::dvec3(-0.177658,  0.061087, 1.02951);
const glm::dvec3 DigibotAnimation::s_naturalRightFootLocal = glm::dvec3( 0.179225,  0.051327, 0.059608);
const glm::dvec3 DigibotAnimation::s_naturalLeftFootLocal  = glm::dvec3(-0.179225,  0.051327, 0.059608);
const glm::dvec3 DigibotAnimation::s_naturalRightHandLocal = glm::dvec3( 0.6,       0.6,      1.4);
const glm::dvec3 DigibotAnimation::s_naturalLeftHandLocal  = glm::dvec3(-0.6,       0.6,      1.4);

// ---- Coordinate helpers ----

glm::dvec3 DigibotAnimation::localToWorld(const glm::dvec3& local, const DigibotAnimationContext& ctx) {
    return ctx.m_digibotWorldOrientation * local + ctx.m_digibotWorldPos;
}

glm::dvec3 DigibotAnimation::worldToLocal(const glm::dvec3& world, const DigibotAnimationContext& ctx) {
    return glm::conjugate(ctx.m_digibotWorldOrientation) * (world - ctx.m_digibotWorldPos);
}

glm::dvec3 DigibotAnimation::worldToGrid(const glm::dvec3& world, const std::shared_ptr<RigidBody>& body) {
    if (!body) return world;
    return glm::conjugate(body->m_orientation) * (world - body->m_position);
}

glm::dvec3 DigibotAnimation::gridToWorld(const glm::dvec3& grid, const std::shared_ptr<RigidBody>& body) {
    if (!body) return grid;
    return body->m_position + body->m_orientation * grid;
}

glm::dvec3 DigibotAnimation::projectOntoPlane(
    const glm::dvec3& point,
    const glm::dvec3& planePoint,
    const glm::dvec3& planeNormal)
{
    // Projection assumes a unit normal; a non-unit/zero normal silently produces garbage.
    assert(isUnit(planeNormal) && "projectOntoPlane requires a unit plane normal");
    return point - glm::dot(point - planePoint, planeNormal) * planeNormal;
}

// ---- Public ----

DigibotPose DigibotAnimation::update(const DigibotAnimationContext& context) {
    if (context.m_mode == DigibotAnimationContext::MovementMode::Walking
        && context.m_hasGroundContact)
    {
        return walkingPose(context);
    }

    m_initialized = false;
    if (context.m_mode == DigibotAnimationContext::MovementMode::Seated) {
        return seatedPose();
    }
    return defaultPose();
}

// ---- Default pose ----

DigibotPose DigibotAnimation::defaultPose() {
    DigibotPose pose;
    pose.rightHand.position = s_naturalRightHandLocal;
    pose.leftHand.position  = s_naturalLeftHandLocal;
    pose.rightFoot.position = s_naturalRightFootLocal;
    pose.leftFoot.position  = s_naturalLeftFootLocal;
    return pose;
}

// ---- Seated pose ----

DigibotPose DigibotAnimation::seatedPose() {
    DigibotPose pose;
    // Feet forward and raised (knees bent as if resting on the cockpit floor),
    // hands forward and lowered onto the controls. All in digibot-local space.
    pose.rightFoot.position = s_naturalRightFootLocal + glm::dvec3{0.0, 0.45, 0.35};
    pose.leftFoot.position  = s_naturalLeftFootLocal  + glm::dvec3{0.0, 0.45, 0.35};
    pose.rightHand.position = s_naturalRightHandLocal + glm::dvec3{-0.15, 0.25, -0.3};
    pose.leftHand.position  = s_naturalLeftHandLocal  + glm::dvec3{ 0.15, 0.25, -0.3};
    return pose;
}

// ---- Walking pose ----

void DigibotAnimation::initFeet(const DigibotAnimationContext& context) {
    auto body = context.m_surfaceBody.lock();

    // Feet start directly below the hips, projected onto the surface plane (grid space).
    glm::dvec3 surfaceGridPosition = worldToGrid(context.m_surfacePoint, body);
    glm::dvec3 upGridDirection = body
        ? glm::conjugate(body->m_orientation) * context.m_surfaceNormal
        : context.m_surfaceNormal;

    auto place = [&](FootState& foot, const glm::dvec3& hipLocal) {
        glm::dvec3 gridPosition = projectOntoPlane(
            worldToGrid(localToWorld(hipLocal, context), body),
            surfaceGridPosition, upGridDirection);
        foot.m_currentGridPosition = foot.m_liftGridPosition
            = foot.m_targetGridPosition = gridPosition;
        foot.m_phase = 1.0;
        foot.m_isPlanted = true;
    };
    place(m_rightFoot, s_naturalRightHipLocal);
    place(m_leftFoot,  s_naturalLeftHipLocal);

    m_surfaceBody = context.m_surfaceBody;
    m_movingFoot  = MovingFoot::NONE;
    m_initialized = true;
}

DigibotPose DigibotAnimation::walkingPose(const DigibotAnimationContext& context) {
    // The controller must hand us a valid, unit surface normal and finite contact point;
    // otherwise the foot-placement projection blows up (see initFeet).
    assert(isUnit(context.m_surfaceNormal) && "surface normal must be unit length");
    assert(isFinite(context.m_surfacePoint) && "surface point must be finite");
    assert(isFinite(context.m_digibotWorldPos) && "digibot world position must be finite");

    auto body        = context.m_surfaceBody.lock();
    auto prevSurface = m_surfaceBody.lock();

    // Grid space is anchored to a specific body; if that changes, its coordinates are stale.
    if (!m_initialized || body.get() != prevSurface.get()) {
        initFeet(context);
    }

    // --- Inputs, all lifted into grid space (the single source of truth) ---
    glm::dvec3 surfaceGridPosition = worldToGrid(context.m_surfacePoint, body);
    glm::dvec3 upGridDirection = body
        ? glm::conjugate(body->m_orientation) * context.m_surfaceNormal
        : context.m_surfaceNormal;

    glm::dvec3 idealRightGridPosition = projectOntoPlane(
        worldToGrid(localToWorld(s_naturalRightHipLocal, context), body),
        surfaceGridPosition, upGridDirection);
    glm::dvec3 idealLeftGridPosition = projectOntoPlane(
        worldToGrid(localToWorld(s_naturalLeftHipLocal, context), body),
        surfaceGridPosition, upGridDirection);

    // --- Velocity relative to the surface, in grid space, tangent to the plane ---
    glm::dvec3 relativeWorldVelocity = context.m_digibotWorldVelocity;
    if (body) {
        relativeWorldVelocity -= body->m_velocity
            + glm::cross(body->getAngularVelocityWorld(),
                         context.m_digibotWorldPos - body->m_position);
    }
    glm::dvec3 gridVelocity = body
        ? glm::conjugate(body->m_orientation) * relativeWorldVelocity
        : relativeWorldVelocity;
    gridVelocity -= glm::dot(gridVelocity, upGridDirection) * upGridDirection;

    // Lead each foot's ideal by half the stance interval, so a landed foot spends
    // its planted time centred around the hip instead of trailing it.
    glm::dvec3 stepLead = gridVelocity * (0.5 * s_stepDuration);
    glm::dvec3 predictedRightGridPosition = idealRightGridPosition + stepLead;
    glm::dvec3 predictedLeftGridPosition  = idealLeftGridPosition + stepLead;

    // --- Trigger a step when a planted foot strays too far from its own ideal.
    // Targets are anchored to the foot's own hip, so feet can't cross.
    if (m_movingFoot == MovingFoot::NONE) {
        double rightError =
            glm::length(m_rightFoot.m_currentGridPosition - predictedRightGridPosition);
        double leftError =
            glm::length(m_leftFoot.m_currentGridPosition - predictedLeftGridPosition);

        if (std::max(rightError, leftError) > s_stepThreshold) {
            m_movingFoot = (rightError > leftError) ? MovingFoot::RIGHT : MovingFoot::LEFT;
            FootState& foot = (m_movingFoot == MovingFoot::RIGHT) ? m_rightFoot : m_leftFoot;
            foot.m_liftGridPosition = foot.m_currentGridPosition;
            foot.m_phase = 0.0;
            foot.m_isPlanted = false;
        }
    }

    // --- Advance the swing: fixed duration, smoothstep travel, sine lift arc ---
    if (m_movingFoot != MovingFoot::NONE) {
        FootState& foot = (m_movingFoot == MovingFoot::RIGHT) ? m_rightFoot : m_leftFoot;

        // Refresh the target from this frame's prediction so it tracks velocity
        // changes, and re-snap both endpoints onto the surface plane: the normal
        // keeps rotating while the foot travels.
        foot.m_targetGridPosition = projectOntoPlane(
            (m_movingFoot == MovingFoot::RIGHT)
                ? predictedRightGridPosition : predictedLeftGridPosition,
            surfaceGridPosition, upGridDirection);
        foot.m_liftGridPosition = projectOntoPlane(
            foot.m_liftGridPosition, surfaceGridPosition, upGridDirection);

        foot.m_phase = std::min(foot.m_phase + context.m_deltaTime / s_stepDuration, 1.0);
        foot.m_currentGridPosition = glm::mix(
                foot.m_liftGridPosition, foot.m_targetGridPosition,
                glm::smoothstep(0.0, 1.0, foot.m_phase))
            + upGridDirection * (s_liftHeight * std::sin(std::numbers::pi * foot.m_phase));

        if (foot.m_phase >= 1.0) {
            foot.m_currentGridPosition = foot.m_targetGridPosition;
            foot.m_isPlanted = true;
            m_movingFoot = MovingFoot::NONE;
        }
    }

    // --- Emit pose: grid space -> world -> Digibot graphics space ---
    DigibotPose pose;
    pose.rightFoot.position =
        worldToLocal(gridToWorld(m_rightFoot.m_currentGridPosition, body), context);
    pose.leftFoot.position =
        worldToLocal(gridToWorld(m_leftFoot.m_currentGridPosition, body), context);
    pose.rightHand.position = s_naturalRightHandLocal;
    pose.leftHand.position  = s_naturalLeftHandLocal;

    // Catch any NaN/inf before it reaches the IK and corrupts the shared render slot.
    assert(isFinite(pose.rightFoot.position) && isFinite(pose.leftFoot.position)
        && "walking pose produced a non-finite foot position");
    return pose;
}
