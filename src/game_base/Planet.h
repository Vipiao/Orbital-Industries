// Planet.h
#pragma once

#include "MeshTransformPublisher.h"
#include "utils/IHashable.h"
#include <glm/glm.hpp>
#include <cstdint>
#include <memory>

class PhysicsEngine;
class GraphicsEngine;
class RigidBody;
class PlanetSurface;
class PlanetType;
struct CdlodInstance;

/**
 * @brief A planet in the world: a rigid body wearing a PlanetType's surface
 *
 * The body is created at the origin with no collider; place and spin it through
 * getRigidBody.
 */
class Planet : public IHashable {
public:
    Planet(uint64_t uniqueId, PhysicsEngine* physics, GraphicsEngine* graphics,
           const PlanetType& type, double massKg);
    ~Planet();

    Planet(const Planet&) = delete;
    Planet& operator=(const Planet&) = delete;

    uint64_t getUniqueId() const { return m_uniqueId; }
    std::weak_ptr<RigidBody> getRigidBody() const { return m_rigidBody; }
    const PlanetSurface& getSurface() const { return *m_surface; }
    // Distance from the centre to the highest the terrain can reach
    double getApproximateRadius() const;

    // Publishes the body's state to graphics
    void updateGraphics(const glm::dvec3& cameraPos);

    // IHashable interface
    virtual size_t computeHash() const override;

private:
    const uint64_t m_uniqueId;
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    std::shared_ptr<const PlanetSurface> m_surface;
    std::weak_ptr<RigidBody> m_rigidBody;
    int m_ssboIndex{-1};
    std::weak_ptr<CdlodInstance> m_cdlodInstance;
    MeshTransformPublisher m_transformPublisher;
};
