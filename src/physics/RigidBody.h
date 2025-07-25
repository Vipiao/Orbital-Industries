// RigidBody.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ColliderReference.h"

// Forward declarations
class Collider;

class RigidBody : public ColliderReference {
public:
    glm::dvec3 m_position;        // Position in world space
    glm::dvec3 m_velocity;        // Linear velocity (already includes deltaTime)
    glm::dvec3 m_forces;          // Accumulated forces
    
    glm::dquat m_orientation;     // Orientation quaternion
    glm::dvec3 m_angularMomentumBody; // Angular momentum L in body space (primary state variable)
    glm::dvec3 m_torques;         // Accumulated torques

    // Cached derived values (mutable for const getters)
    mutable glm::dmat3 m_cachedOrientationMatrix;
    mutable glm::dvec3 m_cachedAngularVelocityBody;
    mutable glm::dvec3 m_cachedAngularVelocityWorld;
    mutable glm::dmat3 m_cachedWorldInertiaTensor;
    mutable glm::dmat3 m_cachedWorldInvInertiaTensor;
    
    // Dirty flags
    mutable bool m_orientationMatrixDirty = true;
    mutable bool m_angularVelocityDirty = true;
    mutable bool m_worldInertiaDirty = true;
    
    double m_mass;                // Mass in kg
    glm::dmat3 m_inertiaTensor;   // Inertia tensor in local space
    double m_invMass;             // Cached inverse mass (1/mass)
    glm::dmat3 m_invInertiaTensor; // Cached inverse inertia tensor in local space
    
    bool m_isStatic;              // If true, this body won't move

    Collider* m_collider;         // Associated collider for collision detection
    glm::dvec3 m_colliderOffset;  // Offset from center of mass to collider origin (in local space)

    // Constructor
    RigidBody() = default;
    
    // Cached getter methods
    const glm::dmat3& getOrientationMatrix() const;
    const glm::dvec3& getAngularVelocityBody() const;
    const glm::dvec3& getAngularVelocityWorld() const;
    void setAngularVelocityBody(glm::dvec3 angularVelocity);
    const glm::dmat3& getWorldInertiaTensor() const;
    const glm::dmat3& getWorldInvInertiaTensor() const;
    
    // Invalidation methods
    void invalidateOrientation();
    void invalidateAngularMomentum();
    void invalidateInertiaTensor();
};