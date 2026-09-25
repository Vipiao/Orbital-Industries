// Planet.cpp
#include "Planet.h"
#include "PlanetType.h"
#include "PlanetWaterGraphics.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "src/world/PlanetSurface.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/SSBOManager.h"
#include <cassert>
#include <cmath>
#include <stdexcept>

Planet::Planet(uint64_t uniqueId, PhysicsEngine* physics, GraphicsEngine* graphics,
               const PlanetType& type, std::weak_ptr<Geometry> waterShell, double massKg)
    : m_uniqueId{uniqueId}, m_physics{physics}, m_graphics{graphics},
      m_surface{type.getSurface()}, m_waterConfig{type.getWater()} {
    if (!m_physics || !m_graphics) {
        throw std::runtime_error("Planet: physics and graphics must be non-null");
    }
    // Zero mass would give zero inertia, which the integrator inverts
    assert(massKg > 0.0 && "planet mass must be positive");

    // A solid sphere of uniform density
    const double radius{m_surface->radius()};
    const double momentOfInertia{0.4 * massKg * radius * radius};
    m_rigidBody = m_physics->addRigidBody(
        glm::dvec3{0.0}, glm::dquat{1.0, 0.0, 0.0, 0.0}, massKg,
        glm::dmat3{momentOfInertia}, false);

    m_ssboIndex = m_graphics->m_ssboManager->allocateIndex();
    m_cdlodInstance = type.createCdlodInstance(m_ssboIndex);
    if (m_waterConfig) {
        assert(m_waterConfig->m_directFalloffMetres > 0.0 &&
               m_waterConfig->m_ambientFalloffMetres > 0.0);
        m_water = std::make_unique<PlanetWaterGraphics>(
            m_graphics, waterShell, m_ssboIndex, seaLevelRadius(), *m_waterConfig);
    }

    // Use a default distant camera position for the initial update
    updateGraphics(glm::dvec3{0.0, 0.0, 100.0});
}

Planet::~Planet() {
    // Both drawables go before the SSBO slot they are drawn through
    m_water.reset();
    m_graphics->removeCdlodInstance(m_cdlodInstance);
    m_graphics->m_ssboManager->deallocateIndex(m_ssboIndex);

    if (!m_rigidBody.expired()) {
        m_physics->removeRigidBody(m_rigidBody);
    }
}

double Planet::getApproximateRadius() const {
    return m_surface->maxRadius();
}

double Planet::seaLevelRadius() const {
    assert(m_waterConfig);
    return m_surface->radius() + m_waterConfig->m_seaLevelMetres;
}

void Planet::updateGraphics(const glm::dvec3& cameraPos) {
    std::shared_ptr<RigidBody> rigidBody{m_rigidBody.lock()};
    if (!rigidBody) {
        return;
    }

    m_transformPublisher.publish(
        m_graphics,
        m_ssboIndex,
        cameraPos,
        rigidBody->getWorldCenterOfMass(),
        rigidBody->getOrientation(),
        rigidBody->m_velocity,
        rigidBody->getAngularVelocityWorld(),
        rigidBody->getCenterOfMassLocal(),
        m_physics->getCurrentPhysicsTimeStep(),
        getApproximateRadius());
}

LightIntensity Planet::lightUnderWater(const glm::dvec3& worldPos) const {
    std::shared_ptr<RigidBody> rigidBody{m_rigidBody.lock()};
    if (!m_waterConfig || !rigidBody) {
        return LightIntensity{};
    }

    // The sea is a sphere about the body's origin, so its orientation is moot
    const double depth{seaLevelRadius() - glm::length(worldPos - rigidBody->getPosition())};
    if (depth <= 0.0) {
        return LightIntensity{};
    }
    return LightIntensity{std::exp(-depth / m_waterConfig->m_ambientFalloffMetres),
                          std::exp(-depth / m_waterConfig->m_directFalloffMetres)};
}

size_t Planet::computeHash() const {
    std::shared_ptr<RigidBody> rigidBody{m_rigidBody.lock()};
    return rigidBody ? rigidBody->computeHash() : 0;
}
