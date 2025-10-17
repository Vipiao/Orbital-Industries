// RigidBody.cpp
#include "RigidBody.h"
#include "../utils/HashFunctions.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

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
    m_angularMomentumBody = m_inertiaTensor * angularVelocity;
    invalidateAngularMomentum();
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

void RigidBody::getInterpolatedTransform(double timeRemainder, glm::dvec3& outPosition, 
                                        glm::dquat& outOrientation) const {
    // Forward interpolation based on current velocity and angular velocity
    // timeRemainder represents the fractional time since the last physics update [0-1]
    
    // Linear position interpolation: p' = p + v * t
    outPosition = m_position + m_velocity * timeRemainder;
    
    // Angular position interpolation using angular velocity
    // We need to construct a quaternion that represents a partial rotation
    const glm::dvec3& angVel = getAngularVelocityWorld();
    double angVelMagnitudeSqr = glm::length2(angVel);
    
    if (angVelMagnitudeSqr > 1e-10) {
        // Create rotation quaternion based on angular velocity
        double angVelMagnitude{ glm::sqrt(angVelMagnitudeSqr) };
        glm::dvec3 axis = angVel / angVelMagnitude;
        double angle = angVelMagnitude * timeRemainder;
        
        // Create rotation quaternion for the partial time step
        glm::dquat partialRotation = glm::angleAxis(angle, axis);
        
        // Apply the partial rotation to the current orientation
        outOrientation = partialRotation * m_orientation;
    } else {
        // No significant rotation, just use the current orientation
        outOrientation = m_orientation;
    }
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

size_t RigidBody::computeHash() const {
    size_t hash = 0;
    
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_position));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_velocity));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_forces));
    hash = Hash::combineHashes(hash, Hash::DQuatHash{}(m_orientation));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_angularMomentumBody));
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_torques));
    hash = Hash::combineHashes(hash, std::hash<double>{}(m_mass));
    hash = Hash::combineHashes(hash, std::hash<bool>{}(m_isStatic));
    // Note: Don't hash cached values or pointers, only core state
    
    return hash;
}

void RigidBody::invalidateInertiaTensor() {
    m_angularVelocityDirty = true;
    m_worldInertiaDirty = true;
}