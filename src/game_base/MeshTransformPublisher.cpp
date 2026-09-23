// MeshTransformPublisher.cpp
#include "MeshTransformPublisher.h"
#include "../physics/RigidBody.h"
#include "graphics/GraphicsEngine.h"
#include <algorithm>

void MeshTransformPublisher::publish(GraphicsEngine* graphics, int ssboIndex,
                                     const glm::dvec3& cameraPos,
                                     const glm::dvec3& position,
                                     const glm::dquat& orientation,
                                     const glm::dvec3& velocity,
                                     const glm::dvec3& angularVelocity,
                                     const glm::dvec3& center,
                                     uint64_t currentTimeStep,
                                     double approximateRadius) {
    if (ssboIndex < 0) {
        return;
    }

    if (!shouldPublish(cameraPos, position, orientation, currentTimeStep,
                       approximateRadius)) {
        return;
    }

    glm::dvec3 angVelAxis{angularVelocity};
    double angVelMagnitude{glm::length(angVelAxis)};
    if (angVelMagnitude > 0.0) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        // No spin: any unit axis will do
        angVelAxis = glm::dvec3{0.0, 0.0, 1.0};
        angVelMagnitude = 0.0;
    }

    graphics->updateMeshTransform(
        ssboIndex,
        position - center,
        velocity,
        orientation,
        angVelAxis,
        angVelMagnitude,
        center,
        glm::dvec3{1.0, 1.0, 1.0},      // scale
        currentTimeStep,
        0.0                             // lit, not emissive
    );

    m_sentPosition = position;
    m_sentVelocity = velocity;
    m_sentOrientation = orientation;
    m_sentAngularVelocity = angularVelocity;
    m_sentTimeStep = currentTimeStep;
    m_nextUpdateTimeStep = currentTimeStep + k_timeThreshold;
}

bool MeshTransformPublisher::shouldPublish(const glm::dvec3& cameraPos,
                                           const glm::dvec3& position,
                                           const glm::dquat& orientation,
                                           uint64_t currentTimeStep,
                                           double approximateRadius) const {
    if (currentTimeStep >= m_nextUpdateTimeStep) {
        return true;
    }

    // Where the GPU draws the body now, extrapolated from what it was sent the
    // same way the engine does
    const double stepsSinceSent{static_cast<double>(currentTimeStep - m_sentTimeStep)};
    const glm::dvec3 predictedPosition{m_sentPosition + m_sentVelocity * stepsSinceSent};
    const glm::dquat predictedOrientation{RigidBody::integrateOrientation(
        m_sentOrientation, m_sentAngularVelocity, stepsSinceSent)};

    // Thresholds grow with distance from the camera to the body's surface
    double distanceToCamera{glm::length(cameraPos - position)};
    // Prevent division by very small numbers
    double effectiveDistance{std::max(distanceToCamera - approximateRadius, 0.1)};
    double scalingFactor{std::max(effectiveDistance, 1.0)};

    double positionDiff{glm::length(position - predictedPosition)};
    if (positionDiff > k_positionThreshold * scalingFactor) {
        return true;
    }

    double orientationDot{glm::abs(glm::dot(orientation, predictedOrientation))};
    // Clamp to handle numerical precision issues
    orientationDot = glm::clamp(orientationDot, 0.0, 1.0);
    double angleDiff{2.0 * glm::acos(orientationDot)};

    // A given angle moves the rim of a larger body further
    double orientationThreshold{k_orientationThresholdBase / approximateRadius * scalingFactor};

    return angleDiff > orientationThreshold;
}
