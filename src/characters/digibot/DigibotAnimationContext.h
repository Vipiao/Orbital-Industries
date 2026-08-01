// DigibotAnimationContext.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <cstdint>

class RigidBody;

struct DigibotAnimationContext {
    enum class MovementMode { Idle, Walking, Flying, Seated };

    MovementMode m_mode{MovementMode::Idle};

    // World transform of the Digibot graphics origin (rigid body pos + graphics offset)
    glm::dvec3 m_digibotWorldPos{0.0, 0.0, 0.0};
    glm::dquat m_digibotWorldOrientation{1.0, 0.0, 0.0, 0.0};

    // World velocity, displacement per physics step (like RigidBody::m_velocity)
    glm::dvec3 m_digibotWorldVelocity{0.0, 0.0, 0.0};

    // Measured frame duration in physics-step units (physicsHz / frameRate).
    // Must track the frame rate actually achieved: the swing advances by this
    // much per frame, so a nominal value ties the swing's real duration to how
    // far the achieved rate happens to sit from it.
    double m_deltaTime{0.0};

    // Ground contact info — valid when m_mode == Walking and m_hasGroundContact == true
    bool m_hasGroundContact{false};
    glm::dvec3 m_surfacePoint{0.0, 0.0, 0.0};
    glm::dvec3 m_surfaceNormal{0.0, 0.0, 1.0};  // character up direction
    std::weak_ptr<RigidBody> m_surfaceBody;      // null for static geometry

    uint64_t m_frameNum{0};
};
