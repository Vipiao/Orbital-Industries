// RigidBody.cpp
#include "RigidBody.h"
#include "utils/HashFunctions.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <cassert>
#include <limits>

// RigidBody cached getter implementations
const glm::dmat3& RigidBody::getOrientationMatrix() const {
    if (m_orientationMatrixDirty) {
        m_cachedOrientationMatrix = glm::mat3_cast(m_orientation);
        m_orientationMatrixDirty = false;
    }
    return m_cachedOrientationMatrix;
}

const glm::dvec3& RigidBody::getAngularVelocityBody() const {
    if (m_angularVelocityBodyDirty) {
        m_cachedAngularVelocityBody = m_invInertiaTensor * m_angularMomentumBody;
        m_angularVelocityBodyDirty = false;
    }
    return m_cachedAngularVelocityBody;
}

const glm::dvec3& RigidBody::getAngularVelocityWorld() const {
    if (m_angularVelocityWorldDirty) {
        m_cachedAngularVelocityWorld = getOrientationMatrix() * getAngularVelocityBody();
        m_angularVelocityWorldDirty = false;
    }
    return m_cachedAngularVelocityWorld;
}

void RigidBody::setAngularVelocityBody(glm::dvec3 angularVelocity) {
    assert(RigidBodyDetail::isFinite(angularVelocity) && "angular velocity must be finite");
    m_angularMomentumBody = m_inertiaTensor * angularVelocity;
    invalidateAngularMomentum();
}

void RigidBody::setMassProperties(double mass, const glm::dvec3& centerOfMassLocal,
                                  const glm::dmat3& inertiaTensor) {
    assert(std::isfinite(mass) && "mass must be finite");
    assert(mass >= 0.0 && "negative mass: a cell was probably removed twice");
    assert(RigidBodyDetail::isFinite(centerOfMassLocal) && "center of mass must be finite");
    assert(RigidBodyDetail::isFinite(inertiaTensor) && "inertia tensor must be finite");
    m_mass = mass;
    m_centerOfMassLocal = centerOfMassLocal;
    m_inertiaTensor = inertiaTensor;
    refreshInverses();
    invalidateInertiaTensor();
    m_worldCenterOfMassDirty = true;
}

void RigidBody::setStatic(bool isStatic) {
    m_isStatic = isStatic;
    refreshInverses();
    invalidateInertiaTensor();
}

void RigidBody::refreshInverses() {
    if (m_isStatic) {
        // Infinite effective mass: impulses and corrections leave the body untouched.
        m_invMass = 0.0;
        m_invInertiaTensor = glm::dmat3{0.0};
        return;
    }
    m_invMass = (m_mass > 1e-15) ? (1.0 / m_mass) : std::numeric_limits<double>::max();
    double determinant{glm::determinant(m_inertiaTensor)};
    m_invInertiaTensor = (determinant > 1e-15)
                             ? glm::inverse(m_inertiaTensor)
                             : glm::dmat3{std::numeric_limits<double>::max()};
}

const glm::dmat3& RigidBody::getWorldInertiaTensor() const {
    if (m_worldInertiaTensorDirty) {
        const glm::dmat3& orientationMatrix = getOrientationMatrix();
        m_cachedWorldInertiaTensor = orientationMatrix * m_inertiaTensor * glm::transpose(orientationMatrix);
        m_worldInertiaTensorDirty = false;
    }
    return m_cachedWorldInertiaTensor;
}

const glm::dmat3& RigidBody::getWorldInvInertiaTensor() const {
    if (m_worldInvInertiaTensorDirty) {
        const glm::dmat3& orientationMatrix = getOrientationMatrix();
        m_cachedWorldInvInertiaTensor = orientationMatrix * m_invInertiaTensor * glm::transpose(orientationMatrix);
        m_worldInvInertiaTensorDirty = false;
    }
    return m_cachedWorldInvInertiaTensor;
}

std::weak_ptr<Collider> RigidBody::getPrimaryCollider() const {
    for (const std::unique_ptr<ColliderAttachment>& attachment : m_attachments) {
        if (!attachment->isTrigger) {
            return attachment->collider;
        }
    }
    return {};
}

glm::dquat RigidBody::integrateOrientation(const glm::dquat& orientation,
                                           const glm::dvec3& angularVelocityWorld,
                                           double dtTicks) {
    double speed{glm::length(angularVelocityWorld)};
    if (speed <= 0.0) {
        return orientation;
    }
    return glm::normalize(
        glm::angleAxis(speed * dtTicks, angularVelocityWorld / speed) * orientation);
}

void RigidBody::rotateAboutCenterOfMass(const glm::dquat& rotation) {
    assert(RigidBodyDetail::isUnitQuaternion(rotation) &&
           "rotation must be normalized or it would rescale the body");
    glm::dvec3 worldCenterOfMass{getWorldCenterOfMass()};
    m_orientation = glm::normalize(rotation * m_orientation);
    invalidateOrientation();  // also dirties the world COM cache for the write below
    m_position = worldCenterOfMass - m_orientation * m_centerOfMassLocal;
}

void RigidBody::getInterpolatedTransform(double timeRemainder, glm::dvec3& outPosition,
                                        glm::dquat& outOrientation) const {
    // timeRemainder: fraction of a tick since the last physics update. The center
    // of mass travels linearly and the body rotates about it; the origin follows.
    outOrientation = integrateOrientation(m_orientation, getAngularVelocityWorld(),
                                          timeRemainder);
    outPosition = getWorldCenterOfMass() + m_velocity * timeRemainder -
                  outOrientation * m_centerOfMassLocal;
}

// RigidBody invalidation methods
void RigidBody::invalidateOrientation() {
    m_orientationMatrixDirty = true;
    m_angularVelocityWorldDirty = true;  // world = R * body; body unchanged
    m_worldInertiaTensorDirty = true;
    m_worldInvInertiaTensorDirty = true;
    m_worldCenterOfMassDirty = true;     // world COM = position + R * local COM
}

void RigidBody::invalidateAngularMomentum() {
    m_angularVelocityBodyDirty = true;
    m_angularVelocityWorldDirty = true;
}

size_t RigidBody::computeHash() const {
    size_t hash = 0;
    
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_position));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_centerOfMassLocal));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_velocity));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_forces));
    hash = Hash::combineHashes(hash, Hash::DQuatHash{}(m_orientation));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_angularMomentumBody));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_torques));
    hash = Hash::combineHashes(hash, std::hash<double>{}(m_mass));
    hash = Hash::combineHashes(hash, std::hash<bool>{}(m_isStatic));

    // Hash attachment count and configuration (not collider pointers themselves)
    hash = Hash::combineHashes(hash, std::hash<size_t>{}(m_attachments.size()));
    for (const auto& attachment : m_attachments) {
        hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(attachment->localPosition));
        hash = Hash::combineHashes(hash, Hash::DQuatHash{}(attachment->localOrientation));
        hash = Hash::combineHashes(hash, std::hash<bool>{}(attachment->isTrigger));
    }
    // Note: Don't hash cached values or pointers, only core state
    
    return hash;
}

void RigidBody::invalidateInertiaTensor() {
    m_angularVelocityBodyDirty = true;
    m_angularVelocityWorldDirty = true;
    m_worldInertiaTensorDirty = true;
    m_worldInvInertiaTensorDirty = true;
}