// RigidBody.cpp
#include "RigidBody.h"
#include <glm/gtc/matrix_transform.hpp>

// RigidBody cached getter implementations
const glm::dmat3& RigidBody::getOrientationMatrix() const {
    if (m_orientationMatrixDirty) {
        m_cachedOrientationMatrix = glm::mat3_cast(m_orientation);
        m_orientationMatrixDirty = false;
    }
    return m_cachedOrientationMatrix;
}

const glm::dvec3& RigidBody::getAngularVelocityBody() const {
    if (m_angularVelocityDirty) {
        m_cachedAngularVelocityBody = m_invInertiaTensor * m_angularMomentumBody;
        m_angularVelocityDirty = false;
    }
    return m_cachedAngularVelocityBody;
}

const glm::dvec3& RigidBody::getAngularVelocityWorld() const {
    if (m_angularVelocityDirty) {
        m_cachedAngularVelocityWorld = getOrientationMatrix() * getAngularVelocityBody();
        m_angularVelocityDirty = false;
    }
    return m_cachedAngularVelocityWorld;
}

void RigidBody::setAngularVelocityBody(glm::dvec3 angularVelocity) {
    glm::dvec3 localAngularVelocity = glm::transpose(getOrientationMatrix()) * angularVelocity;
    m_angularMomentumBody = m_inertiaTensor * localAngularVelocity;
}

const glm::dmat3& RigidBody::getWorldInertiaTensor() const {
    if (m_worldInertiaDirty) {
        const glm::dmat3& orientationMatrix = getOrientationMatrix();
        m_cachedWorldInertiaTensor = orientationMatrix * m_inertiaTensor * glm::transpose(orientationMatrix);
        m_worldInertiaDirty = false;
    }
    return m_cachedWorldInertiaTensor;
}

const glm::dmat3& RigidBody::getWorldInvInertiaTensor() const {
    if (m_worldInertiaDirty) {
        const glm::dmat3& orientationMatrix = getOrientationMatrix();
        m_cachedWorldInvInertiaTensor = orientationMatrix * m_invInertiaTensor * glm::transpose(orientationMatrix);
        m_worldInertiaDirty = false;
    }
    return m_cachedWorldInvInertiaTensor;
}

// RigidBody invalidation methods
void RigidBody::invalidateOrientation() {
    m_orientationMatrixDirty = true;
    m_angularVelocityDirty = true;
    m_worldInertiaDirty = true;
}

void RigidBody::invalidateAngularMomentum() {
    m_angularVelocityDirty = true;
}

void RigidBody::invalidateInertiaTensor() {
    m_angularVelocityDirty = true;
    m_worldInertiaDirty = true;
}