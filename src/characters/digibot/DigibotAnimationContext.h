// DigibotAnimationContext.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <cstdint>

class RigidBody;

struct DigibotAnimationContext {
    enum class MovementMode { Idle, Walking, Flying };

    MovementMode m_mode{MovementMode::Idle};

    // World transform of the Digibot graphics origin (rigid body pos + graphics offset)
    glm::dvec3 m_digibotWorldPos{0.0, 0.0, 0.0};
    glm::dquat m_digibotWorldOrientation{1.0, 0.0, 0.0, 0.0};

    // Ground contact info — valid when m_mode == Walking and m_hasGroundContact == true
    bool m_hasGroundContact{false};
    glm::dvec3 m_surfacePoint{0.0, 0.0, 0.0};
    glm::dvec3 m_surfaceNormal{0.0, 0.0, 1.0};  // character up direction
    std::weak_ptr<RigidBody> m_surfaceBody;      // null for static geometry

    uint64_t m_frameNum{0};
};
