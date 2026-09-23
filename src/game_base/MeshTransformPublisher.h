// MeshTransformPublisher.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>

class GraphicsEngine;

/**
 * @brief Publishes a body's transform to one SSBO slot, only when it matters
 *
 * The GPU extrapolates from the last published state. This keeps that state,
 * extrapolates it the same way, and publishes again once the real state drifts
 * from it by more than a camera-distance scaled threshold, or after a fixed
 * number of steps.
 */
class MeshTransformPublisher {
public:
    // position is the world centre of mass, center the centre of mass in the
    // body's local frame.
    void publish(GraphicsEngine* graphics, int ssboIndex,
                 const glm::dvec3& cameraPos,
                 const glm::dvec3& position,
                 const glm::dquat& orientation,
                 const glm::dvec3& velocity,
                 const glm::dvec3& angularVelocity,
                 const glm::dvec3& center,
                 uint64_t currentTimeStep,
                 double approximateRadius);

private:
    bool shouldPublish(const glm::dvec3& cameraPos,
                       const glm::dvec3& position,
                       const glm::dquat& orientation,
                       uint64_t currentTimeStep,
                       double approximateRadius) const;

    // What the GPU was last sent, which it extrapolates from
    glm::dvec3 m_sentPosition{0.0};
    glm::dvec3 m_sentVelocity{0.0};
    glm::dquat m_sentOrientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 m_sentAngularVelocity{0.0};
    uint64_t m_sentTimeStep{0};
    uint64_t m_nextUpdateTimeStep{0};

    static constexpr double k_positionThreshold{0.0008};
    static constexpr double k_orientationThresholdBase{0.0004};
    static constexpr uint64_t k_timeThreshold{256};
};
