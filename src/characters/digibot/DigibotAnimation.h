// DigibotAnimation.h
#pragma once

#include "DigibotPose.h"
#include "DigibotAnimationContext.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

class RigidBody;

class DigibotAnimation {
public:
    DigibotAnimation() = default;

    DigibotPose update(const DigibotAnimationContext& context);

private:
    static DigibotPose defaultPose();
    DigibotPose walkingPose(const DigibotAnimationContext& context);

    // Skeleton constants in Digibot-local space (from DigibotGraphics measurements)
    static const glm::dvec3 s_naturalRightHipLocal;
    static const glm::dvec3 s_naturalLeftHipLocal;
    static const glm::dvec3 s_naturalRightFootLocal;
    static const glm::dvec3 s_naturalLeftFootLocal;
    static const glm::dvec3 s_naturalRightHandLocal;
    static const glm::dvec3 s_naturalLeftHandLocal;

    // All foot state lives in surface-body (grid) space — the single source of truth.
    // A planted foot is simply a fixed point there, so it follows the grid for free.
    // A stepping foot chases its landing spot through an intermediate point:
    //   intermediate eases toward target, current eases toward intermediate
    //   (double exponential for a soft step arc).
    struct FootState {
        glm::dvec3 m_currentGridPosition{0.0, 0.0, 0.0};
        glm::dvec3 m_intermediateGridPosition{0.0, 0.0, 0.0};
        glm::dvec3 m_targetGridPosition{0.0, 0.0, 0.0};
        bool m_isPlanted{true};
    };

    enum class MovingFoot { NONE, LEFT, RIGHT };

    FootState m_leftFoot;
    FootState m_rightFoot;
    MovingFoot m_movingFoot{MovingFoot::NONE};
    std::weak_ptr<RigidBody> m_surfaceBody;
    bool m_initialized{false};

    static constexpr double s_stepThreshold = 0.30;
    static constexpr double s_footAlpha    = 0.20;
    static constexpr double s_plantMargin  = 0.02;

    // Digibot-local <-> world (uses the context's digibot transform).
    static glm::dvec3 localToWorld(const glm::dvec3& local, const DigibotAnimationContext& ctx);
    static glm::dvec3 worldToLocal(const glm::dvec3& world, const DigibotAnimationContext& ctx);

    // World <-> surface-body (grid) space. A null body means grid space == world space.
    static glm::dvec3 worldToGrid(const glm::dvec3& world, const std::shared_ptr<RigidBody>& body);
    static glm::dvec3 gridToWorld(const glm::dvec3& grid, const std::shared_ptr<RigidBody>& body);

    static glm::dvec3 projectOntoPlane(
        const glm::dvec3& point,
        const glm::dvec3& planePoint,
        const glm::dvec3& planeNormal);

    void initFeet(const DigibotAnimationContext& context);
};
