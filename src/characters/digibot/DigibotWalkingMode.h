// DigibotWalkingMode.h
#pragma once

#include "DigibotModeTypes.h"
#include "../../physics/PhysicsUnits.h"
#include <glm/glm.hpp>
#include <memory>

class DigibotPhysics;
class PhysicsEngine;
class RigidBody;

// Ground movement: contact selection from the walking sensor, hover height control,
// surface-relative locomotion and orientation alignment. Pure wrench producer — the
// controller applies the result (with the equal-opposite reaction on the ground body).
class DigibotWalkingMode {
public:
    DigibotWalkingMode(DigibotPhysics* physics, PhysicsEngine* physicsEngine);

    DigibotWrench update(const std::shared_ptr<RigidBody>& rigidBody,
                         const DigibotModeInputs& inputs);

    // Clear transient contact state (used when leaving walking or entering docking).
    void resetContactState();

    // Ground contact info for animation (valid only while walking). The contact
    // is cached in the ground body's local frame, so these return world-space
    // values at the body's transform as of the call -- readers after physics
    // integration see the contact where the moving ground actually is.
    bool hasGroundContact() const { return m_hasGroundContact; }
    glm::dvec3 getGroundContactPoint() const;
    glm::dvec3 getGroundSurfaceNormal() const;
    std::weak_ptr<RigidBody> getWalkingTargetRigidBody() const {
        return m_walkingTargetRigidBody;
    }

private:
    DigibotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;

    // Movement parameters
    double m_angularAccelerationMax{PhysicsUnits::radiansPerSecondSquared(65.536)};
    double m_targetHoverHeight{1.0};
    double m_maxGroundAcceleration{PhysicsUnits::metersPerSecondSquared(32.768)};
    double m_targetWalkSpeed{PhysicsUnits::metersPerSecond(5.12)};
    double m_walkingThrustStrength{PhysicsUnits::metersPerSecondSquared(28.672)};
    double m_groundSelectionBias{2.0};
    double m_maxGroundAngle{glm::radians(90.0)};
    double m_maxLockedGroundAngle{glm::radians(30.0)};

    // Frames spent without ground contact (physics timestep units)
    uint64_t m_framesWithoutContact{0};

    // Target rigid body from walking (empty if no contact)
    std::weak_ptr<RigidBody> m_walkingTargetRigidBody{};

    // Ground contact state (updated each stepControl). In the target body's
    // local frame when there is a target body, world frame for static ground.
    bool m_hasGroundContact{false};
    glm::dvec3 m_groundContactPointLocal{0.0, 0.0, 0.0};
    // Direction toward the body from the contact, independent of any locked up.
    glm::dvec3 m_groundSurfaceNormalLocal{0.0, 0.0, 1.0};

    // Up direction lock cache (in rigid body local coordinates of the cached body)
    glm::dvec3 m_cachedModifiedUp{0.0, 0.0, 0.0};
    std::weak_ptr<RigidBody> m_cachedRigidBody{};
};
