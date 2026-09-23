// PlanetSubsystem.h
#pragma once

#include "utils/IHashable.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>

class PhysicsEngine;
class GraphicsEngine;
class Planet;
class PlanetType;
struct PlanetTypeConfig;

/**
 * @brief Owns every PlanetType and Planet, and publishes the planets' state to
 *        graphics each step
 *
 * Parallel to GridSubsystem in architecture.
 */
class PlanetSubsystem : public IHashable {
public:
    PlanetSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics);
    ~PlanetSubsystem();

    // Generates the type's maps; the expensive part, done once per type
    std::weak_ptr<const PlanetType> createPlanetType(const PlanetTypeConfig& config);
    std::weak_ptr<Planet> createPlanet(std::weak_ptr<const PlanetType> type, double massKg);

    // Once per physics step, after integration
    void stepUpdateGraphicsAll(const glm::dvec3& cameraPos);

    // IHashable interface
    virtual size_t computeHash() const override;

private:
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;

    // Declared before m_planets so every type outlives the planets wearing it
    std::vector<std::shared_ptr<const PlanetType>> m_planetTypes;

    // In creation order, which is the same on every peer
    std::vector<std::shared_ptr<Planet>> m_planets;
};
