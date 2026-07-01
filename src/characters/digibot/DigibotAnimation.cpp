// DigibotAnimation.cpp
#include "DigibotAnimation.h"
#include "../../physics/RigidBody.h"

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
        foot.m_currentGridPosition = foot.m_intermediateGridPosition
            = foot.m_targetGridPosition = gridPosition;
        foot.m_isPlanted = true;
    };
    place(m_rightFoot, s_naturalRightHipLocal);
    place(m_leftFoot,  s_naturalLeftHipLocal);

    m_surfaceBody = context.m_surfaceBody;
    m_movingFoot  = MovingFoot::NONE;
    m_initialized = true;
}

DigibotPose DigibotAnimation::walkingPose(const DigibotAnimationContext& context) {
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
    glm::dvec3 digibotGridPosition = worldToGrid(context.m_digibotWorldPos, body);

    glm::dvec3 idealRightGridPosition = projectOntoPlane(
        worldToGrid(localToWorld(s_naturalRightHipLocal, context), body),
        surfaceGridPosition, upGridDirection);
    glm::dvec3 idealLeftGridPosition = projectOntoPlane(
        worldToGrid(localToWorld(s_naturalLeftHipLocal, context), body),
        surfaceGridPosition, upGridDirection);

    // --- Trigger a step when both feet are planted and their average drifts off-center ---
    if (m_movingFoot == MovingFoot::NONE) {
        glm::dvec3 underneathGridPosition =
            projectOntoPlane(digibotGridPosition, surfaceGridPosition, upGridDirection);
        glm::dvec3 averageFootGridPosition =
            (m_leftFoot.m_currentGridPosition + m_rightFoot.m_currentGridPosition) * 0.5;

        if (glm::length(averageFootGridPosition - underneathGridPosition) > s_stepThreshold) {
            double rightError =
                glm::length(m_rightFoot.m_currentGridPosition - idealRightGridPosition);
            double leftError =
                glm::length(m_leftFoot.m_currentGridPosition - idealLeftGridPosition);
            MovingFoot mover = (rightError > leftError) ? MovingFoot::RIGHT : MovingFoot::LEFT;

            FootState& moverFoot  = (mover == MovingFoot::RIGHT) ? m_rightFoot : m_leftFoot;
            FootState& anchorFoot = (mover == MovingFoot::RIGHT) ? m_leftFoot  : m_rightFoot;

            // Mirror the anchor through underneath so the pair re-centres under the body,
            // then project onto the surface: a point reflection would otherwise flip the
            // anchor's out-of-plane component and place the target off the surface.
            moverFoot.m_targetGridPosition = projectOntoPlane(
                2.0 * underneathGridPosition - anchorFoot.m_currentGridPosition,
                surfaceGridPosition, upGridDirection);
            moverFoot.m_isPlanted = false;
            m_movingFoot = mover;
        }
    }

    // --- Animate the moving foot: intermediate eases toward target, current toward intermediate ---
    if (m_movingFoot != MovingFoot::NONE) {
        FootState& foot = (m_movingFoot == MovingFoot::RIGHT) ? m_rightFoot : m_leftFoot;

        // Re-snap the target onto the surface each frame: the normal keeps rotating while
        // the foot travels, so a target frozen in grid space would drift off the plane.
        foot.m_targetGridPosition = projectOntoPlane(
            foot.m_targetGridPosition, surfaceGridPosition, upGridDirection);

        foot.m_intermediateGridPosition +=
            (foot.m_targetGridPosition - foot.m_intermediateGridPosition) * s_footAlpha;
        foot.m_currentGridPosition +=
            (foot.m_intermediateGridPosition - foot.m_currentGridPosition) * s_footAlpha;

        if (glm::length(foot.m_currentGridPosition - foot.m_targetGridPosition) < s_plantMargin) {
            foot.m_currentGridPosition = foot.m_intermediateGridPosition
                = foot.m_targetGridPosition;
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
    return pose;
}
