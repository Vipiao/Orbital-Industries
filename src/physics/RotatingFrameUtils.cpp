// RotatingFrameUtils.cpp
#include "RotatingFrameUtils.h"
#include "RigidBody.h"

namespace RotatingFrameUtils {

glm::dvec3 centrifugalCoriolisCompensation(double mass,
                                           const glm::dvec3& angularVelocity,
                                           const glm::dvec3& radius,
                                           const glm::dvec3& relativeVelocity) {
    glm::dvec3 centrifugalForce{
        -mass * glm::cross(angularVelocity, glm::cross(angularVelocity, radius))};
    glm::dvec3 coriolisForce{
        -2.0 * mass * glm::cross(angularVelocity, relativeVelocity)};
    return -centrifugalForce - coriolisForce;
}

double effectiveMass(const RigidBody& body, const RigidBody* other,
                     const glm::dvec3& direction, const glm::dvec3& reactionOffset) {
    if (!other) {
        return body.getMass();
    }
    // Force at `body`'s COM has no rotational contribution for `body` itself.
    glm::dvec3 rCrossDir{glm::cross(reactionOffset, direction)};
    glm::dvec3 rotContribution{other->getWorldInvInertiaTensor() * rCrossDir};
    double rotTerm{glm::dot(rCrossDir, rotContribution)};
    double invEffectiveMass{body.getInvMass() + other->getInvMass() + rotTerm};
    return 1.0 / invEffectiveMass;
}

glm::dmat3 effectiveInertia(const RigidBody& body, const RigidBody* other) {
    if (!other || other->isStatic()) {
        return body.getWorldInertiaTensor();
    }
    glm::dmat3 invInertiaSum{body.getWorldInvInertiaTensor() +
                             other->getWorldInvInertiaTensor()};
    return glm::inverse(invInertiaSum);
}

}
