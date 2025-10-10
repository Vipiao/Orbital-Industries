// src/characters/ArticulationUtils.cpp
#include "ArticulationUtils.h"
#include "../graphics/instanceHandler/InstanceHandler.h"
#include <glm/gtx/quaternion.hpp>

glm::dvec3 ArticulationUtils::applyDistanceConstraint(const glm::dvec3& point, const glm::dvec3& anchor, double targetDistance) {
    glm::dvec3 direction = point - anchor;
    double currentDistance = glm::length(direction);
    
    if (currentDistance > 1e-6) {
        return anchor + glm::normalize(direction) * targetDistance;
    } else {
        // If point is exactly on anchor, pick arbitrary direction
        return anchor + glm::dvec3(1.0, 0.0, 0.0) * targetDistance;
    }
}

glm::dvec3 ArticulationUtils::applyDirectionNudge(const glm::dvec3& point, const glm::dvec3& preferredDirection, double nudgeStrength) {
    return point + preferredDirection * nudgeStrength;
}

glm::dquat ArticulationUtils::quatLookAtYForward(const glm::dvec3& direction, const glm::dvec3& up) {
    // Direction should already be normalized
    glm::dvec3 forward = direction;
    
    // Calculate right vector (perpendicular to forward and up)
    glm::dvec3 right = glm::normalize(glm::cross(forward, up));
    
    // Recalculate up to ensure orthogonality
    glm::dvec3 upAdjusted = glm::cross(right, forward);
    
    // Build rotation matrix where:
    // - Column 0 (+X axis) points in right direction
    // - Column 1 (+Y axis) points in forward direction  
    // - Column 2 (+Z axis) points in up direction
    glm::dmat3 rotationMatrix(
        right.x, forward.x, upAdjusted.x,
        right.y, forward.y, upAdjusted.y,
        right.z, forward.z, upAdjusted.z
    );
    
    // Convert to quaternion
    return glm::quat_cast(rotationMatrix);
}

void ArticulationUtils::updateArticulatedBodyPart(
    std::weak_ptr<Geometry> geometry,
    std::weak_ptr<Instance> instance,
    const glm::dvec3& directionFrom,
    const glm::dvec3& directionTo,
    const glm::dvec3& upVector,
    const glm::dquat& correctionQuat,
    const glm::dvec3& naturalPivot)
{
    auto geometryLocked = geometry.lock();
    auto instanceLocked = instance.lock();
    
    if (!geometryLocked || !instanceLocked) {
        return;
    }
    
    // Calculate direction the limb should point
    glm::dvec3 direction = glm::normalize(directionTo - directionFrom);
    
    // Build orientation quaternion
    // quatLookAtYForward makes +Y point along direction
    glm::dquat lookAtQuat = ArticulationUtils::quatLookAtYForward(direction, upVector);
    
    // Invert because lookAt is for cameras (inverse of object orientation)
    glm::dquat invertedLookAt = glm::inverse(lookAtQuat);
    
    // Apply model-specific correction (e.g., if model points along X instead of Y)
    glm::dquat finalOrientation = invertedLookAt * correctionQuat;
    
    // Calculate how much the pivot has moved from its natural position
    glm::dvec3 pivotShift = directionFrom - naturalPivot;
    
    // Calculate rotation shift: how much the natural pivot moves when rotated
    glm::dvec3 rotationShift = finalOrientation * naturalPivot - naturalPivot;
    
    // Set final transform
    instanceLocked->m_localOrientation = finalOrientation;
    instanceLocked->m_localPosition = -rotationShift + pivotShift;
    
    // Update GPU buffer
    geometryLocked->updateInstanceInBuffer(instanceLocked.get());
}