// RotatingFrameUtils.h
#pragma once

#include <glm/glm.hpp>

class RigidBody;

// Stateless helpers for movement relative to bodies that translate and rotate
// (grids). Leaf module: depends only on read access to RigidBody.
namespace RotatingFrameUtils {

// Feed-forward force cancelling centrifugal and coriolis pseudo-forces for a mass
// moving with `relativeVelocity` at offset `radius` from the rotation centre of a
// frame rotating at `angularVelocity`.
glm::dvec3 centrifugalCoriolisCompensation(double mass,
                                           const glm::dvec3& angularVelocity,
                                           const glm::dvec3& radius,
                                           const glm::dvec3& relativeVelocity);

// Effective mass for a force applied along `direction` (unit) at `body`'s centre of
// mass with the equal-opposite reaction applied to `other` at offset
// `reactionOffset` from `other`'s centre of mass. `other` may be null (infinite
// environment), in which case only `body`'s mass counts.
double effectiveMass(const RigidBody& body, const RigidBody* other,
                     const glm::dvec3& direction, const glm::dvec3& reactionOffset);

// Combined rotational inertia for a torque exchanged between two bodies. `other`
// may be null or static, in which case `body`'s world inertia tensor is returned.
glm::dmat3 effectiveInertia(const RigidBody& body, const RigidBody* other);

}
