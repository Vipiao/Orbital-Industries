// RigidBody.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>
#include "utils/IHashable.h"

// Forward declarations
class RigidBody;
class Collider;

// Finiteness predicates for the state assertions below. A NaN entering the body
// state is silent until it has spread through the whole simulation, so every
// mutation path checks its input at the point the bad value would enter.
namespace RigidBodyDetail {

inline bool isFinite(const glm::dvec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline bool isFinite(const glm::dquat& value) {
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

inline bool isFinite(const glm::dmat3& value) {
    for (int column{0}; column < 3; ++column) {
        for (int row{0}; row < 3; ++row) {
            if (!std::isfinite(value[column][row])) {
                return false;
            }
        }
    }
    return true;
}

// The centre of mass is placed by rotating an offset with the orientation, so a
// quaternion that has drifted off the unit sphere would scale that offset.
inline bool isUnitQuaternion(const glm::dquat& value) {
    return std::abs(glm::length(value) - 1.0) < 1e-9;
}

}  // namespace RigidBodyDetail

/**
 * @brief Attachment configuration for a collider connected to a rigid body
 *
 * Defines how a collider moves with and affects a rigid body.
 */
struct ColliderAttachment {
    std::weak_ptr<RigidBody> rigidBody;  // Back-reference to owning rigid body
    std::weak_ptr<Collider> collider;    // The attached collider
    glm::dvec3 localPosition;            // Collider position relative to body position (body space)
    glm::dquat localOrientation;         // Orientation offset in body space
    bool isTrigger;                      // If true, detects collisions but doesn't affect momentum
};

class RigidBody : public IHashable {
public:
    // Directly writable state: no cached value derives from these.
    glm::dvec3 m_velocity{0.0, 0.0, 0.0};  // Center-of-mass velocity (includes deltaTime)
    glm::dvec3 m_forces{0.0, 0.0, 0.0};    // Accumulated forces
    glm::dvec3 m_torques{0.0, 0.0, 0.0};   // Accumulated torques

    std::vector<std::unique_ptr<ColliderAttachment>> m_attachments; // Attached colliders

    RigidBody() = default;

    // ===== Core state. The fields live in the private section; every write runs
    // through a setter so the cached derived values (orientation matrix, angular
    // velocities, world inertia, world center of mass) can never go stale. =====

    // Origin of the body frame in world space. A bookkeeping anchor, not the center
    // of mass: it stays put when mass is redistributed (e.g. a grid losing blocks),
    // so local offsets stored against it stay valid for the body's lifetime.
    const glm::dvec3& getPosition() const { return m_position; }
    void setPosition(const glm::dvec3& position) {
        assert(RigidBodyDetail::isFinite(position) && "rigid body position must be finite");
        m_position = position;
        m_worldCenterOfMassDirty = true;
    }

    // Raw orientation write: rotates the body about its origin, so the world center
    // of mass moves with it. To rotate in place, use rotateAboutCenterOfMass.
    const glm::dquat& getOrientation() const { return m_orientation; }
    void setOrientation(const glm::dquat& orientation) {
        assert(RigidBodyDetail::isFinite(orientation) &&
               "rigid body orientation must be finite");
        assert(RigidBodyDetail::isUnitQuaternion(orientation) &&
               "orientation must be normalized; it rotates the center-of-mass offset");
        m_orientation = orientation;
        invalidateOrientation();
    }

    // If true, this body won't move: zero inverse mass and inertia, so impulses and
    // positional corrections leave it untouched.
    bool isStatic() const { return m_isStatic; }
    void setStatic(bool isStatic);

    // Center of mass relative to the position, in the body's local frame. Dynamics
    // (momentum, torque arms, inertia) are about the center of mass, never the
    // origin. Written only through setMassProperties.
    const glm::dvec3& getCenterOfMassLocal() const { return m_centerOfMassLocal; }

    // Angular momentum L in body space (primary state variable).
    const glm::dvec3& getAngularMomentumBody() const { return m_angularMomentumBody; }
    void setAngularMomentumBody(const glm::dvec3& angularMomentumBody) {
        assert(RigidBodyDetail::isFinite(angularMomentumBody) &&
               "angular momentum must be finite");
        m_angularMomentumBody = angularMomentumBody;
        invalidateAngularMomentum();
    }
    void setAngularVelocityBody(glm::dvec3 angularVelocity);

    double getMass() const { return m_mass; }
    double getInvMass() const { return m_invMass; }
    // Inertia tensor in local space, about the center of mass.
    const glm::dmat3& getInertiaTensor() const { return m_inertiaTensor; }
    const glm::dmat3& getInvInertiaTensor() const { return m_invInertiaTensor; }

    // Sets mass, local center of mass and inertia as one unit, refreshing the
    // cached inverses. A static body keeps zero inverses so impulses and
    // corrections leave it untouched.
    void setMassProperties(double mass, const glm::dvec3& centerOfMassLocal,
                           const glm::dmat3& inertiaTensor);

    // Cached getter methods
    const glm::dmat3& getOrientationMatrix() const;
    const glm::dvec3& getAngularVelocityBody() const;
    const glm::dvec3& getAngularVelocityWorld() const;

    // Center of mass in world space: the point the dynamics revolve around.
    const glm::dvec3& getWorldCenterOfMass() const {
        if (m_worldCenterOfMassDirty) {
            m_cachedWorldCenterOfMass =
                m_position + getOrientationMatrix() * m_centerOfMassLocal;
            m_worldCenterOfMassDirty = false;
        }
        return m_cachedWorldCenterOfMass;
    }

    // World-space velocity of the material point at worldPoint on this body.
    glm::dvec3 velocityAtPoint(const glm::dvec3& worldPoint) const {
        return m_velocity +
               glm::cross(getAngularVelocityWorld(), worldPoint - getWorldCenterOfMass());
    }

    // Applies a world-space rotation about the center of mass, moving the origin so
    // the center of mass stays fixed. Any in-place orientation change must pivot
    // here, not at the origin, or it would translate the center of mass.
    void rotateAboutCenterOfMass(const glm::dquat& rotation);
    const glm::dmat3& getWorldInertiaTensor() const;
    const glm::dmat3& getWorldInvInertiaTensor() const;

    // The body's primary collider: its first non-trigger attachment, or empty if it
    // has none. A trigger (e.g. a sensor) detects contacts but doesn't represent the
    // body itself, so it is skipped.
    std::weak_ptr<Collider> getPrimaryCollider() const;

    // The orientation advanced dtTicks under a constant world-space angular
    // velocity (radians per tick). dtTicks may be negative or fractional.
    static glm::dquat integrateOrientation(const glm::dquat& orientation,
                                           const glm::dvec3& angularVelocityWorld,
                                           double dtTicks);

    // Interpolation for smooth rendering
    void getInterpolatedTransform(double timeRemainder, glm::dvec3& outPosition,
                                 glm::dquat& outOrientation) const;

    // IHashable interface
    virtual size_t computeHash() const override;

private:
    // Invalidation is internal: the setters above are the only mutation paths, so
    // callers never manage cache consistency themselves.
    void invalidateOrientation();
    void invalidateAngularMomentum();
    void invalidateInertiaTensor();
    // Recomputes the inverses from the current mass, inertia and static flag, so
    // setting those in either order leaves the same consistent state.
    void refreshInverses();

    glm::dvec3 m_position{0.0, 0.0, 0.0};
    glm::dvec3 m_centerOfMassLocal{0.0, 0.0, 0.0};
    glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};  // Orientation quaternion
    glm::dvec3 m_angularMomentumBody{0.0, 0.0, 0.0};  // Angular momentum L in body space
    double m_mass{0.0};                     // Mass in kg
    glm::dmat3 m_inertiaTensor{0.0};        // Inertia tensor in local space
    double m_invMass{0.0};                  // Cached inverse mass (1/mass)
    glm::dmat3 m_invInertiaTensor{0.0};     // Cached inverse inertia tensor in local space
    bool m_isStatic{false};

    // Cached derived values (mutable for const getters)
    mutable glm::dmat3 m_cachedOrientationMatrix;
    mutable glm::dvec3 m_cachedAngularVelocityBody;
    mutable glm::dvec3 m_cachedAngularVelocityWorld;
    mutable glm::dmat3 m_cachedWorldInertiaTensor;
    mutable glm::dmat3 m_cachedWorldInvInertiaTensor;
    mutable glm::dvec3 m_cachedWorldCenterOfMass;

    // Dirty flags
    mutable bool m_orientationMatrixDirty{true};
    mutable bool m_angularVelocityBodyDirty{true};
    mutable bool m_angularVelocityWorldDirty{true};
    mutable bool m_worldInertiaTensorDirty{true};
    mutable bool m_worldInvInertiaTensorDirty{true};
    mutable bool m_worldCenterOfMassDirty{true};
};
