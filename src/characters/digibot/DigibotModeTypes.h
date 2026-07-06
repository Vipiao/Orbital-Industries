// DigibotModeTypes.h
#pragma once

#include <glm/glm.hpp>
#include <memory>

class RigidBody;

// Lock state shared by controller and movement modes.
enum class DigibotLockState { UNLOCKED, TRANSLATION_LOCK, FULL_LOCK };

// Per-step inputs shared by all movement modes (world space where applicable).
struct DigibotModeInputs {
    glm::ivec3 m_movementDirection{0, 0, 0};
    glm::dvec3 m_viewDirection{0.0, 1.0, 0.0};
    int m_rollInput{0};
    bool m_isUpDirectionLocked{false};
};

// Force/torque a movement mode wants applied to the digibot this step. If
// m_reactionBody is set, the orchestrator applies the equal and opposite wrench to
// it at the digibot's position so momentum and angular momentum are conserved.
struct DigibotWrench {
    glm::dvec3 m_force{0.0, 0.0, 0.0};
    glm::dvec3 m_torque{0.0, 0.0, 0.0};
    std::weak_ptr<RigidBody> m_reactionBody{};
};
