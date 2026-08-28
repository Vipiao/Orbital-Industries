// RigidBodyState.h — the state an integrator carries from one tick to the next.
// Capturing and restoring it reproduces a body exactly: a natural unit for
// snapshots, history buffers, and save games. The physics engine itself never
// consumes it; layers above capture, ship or store body state through it.
#pragma once

#include "utils/ByteStream.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

class PhysicsEngine;
class RigidBody;

struct RigidBodyState {
    glm::dvec3 m_position{0.0, 0.0, 0.0};
    glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 m_velocity{0.0, 0.0, 0.0};
    glm::dvec3 m_angularMomentumBody{0.0, 0.0, 0.0};

    static RigidBodyState capture(const RigidBody& body);

    // Writes the state into the body and refreshes everything derived from it:
    // the cached rotation values and the collider transform.
    void apply(const std::weak_ptr<RigidBody>& bodyWeak, PhysicsEngine& physicsEngine) const;

    void serialize(ByteWriter& writer) const;
    bool deserialize(ByteReader& reader);

    // A non-finite value poisons the integrator for good, and an orientation off the
    // unit sphere rescales whatever it rotates, so decoded or loaded state is screened
    // with this before being applied.
    bool isValid() const;
};
