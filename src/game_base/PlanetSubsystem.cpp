// PlanetSubsystem.cpp
#include "PlanetSubsystem.h"
#include "Planet.h"
#include "PlanetType.h"
#include "utils/HashFunctions.h"
#include "graphics/GraphicsEngine.h"
#include <stdexcept>

PlanetSubsystem::PlanetSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics)
    : m_physics{physics}, m_graphics{graphics} {
    if (!m_physics || !m_graphics) {
        throw std::runtime_error("PlanetSubsystem: All dependencies must be non-null");
    }

    // A coarse icosphere, since the shader rather than the mesh shapes the water
    const size_t waterMaterial{
        m_graphics->createRayVolumeMaterial("../media/planet/water_body.glsl")};
    m_waterShell = m_graphics->createRayVolumeGeometry(
        "../media/blender/02_sphere.obj", waterMaterial);
}

PlanetSubsystem::~PlanetSubsystem() {
    // The planets' water instances go before the shell they are drawn on
    m_planets.clear();
    m_graphics->releaseRayVolumeGeometry(m_waterShell);
}

std::weak_ptr<const PlanetType> PlanetSubsystem::createPlanetType(
    const PlanetTypeConfig& config) {
    m_planetTypes.push_back(std::make_shared<const PlanetType>(m_graphics, config));
    return m_planetTypes.back();
}

std::weak_ptr<Planet> PlanetSubsystem::createPlanet(std::weak_ptr<const PlanetType> type,
                                                    double massKg) {
    const std::shared_ptr<const PlanetType> lockedType{type.lock()};
    if (!lockedType) {
        throw std::runtime_error("PlanetSubsystem::createPlanet: type has expired");
    }
    m_planets.push_back(std::make_shared<Planet>(m_nextPlanetId++, m_physics, m_graphics,
                                                 *lockedType, m_waterShell, massKg));
    return m_planets.back();
}

std::weak_ptr<Planet> PlanetSubsystem::getPlanetById(uint64_t id) const {
    for (const std::shared_ptr<Planet>& planet : m_planets) {
        if (planet->getUniqueId() == id) {
            return planet;
        }
    }
    return {};
}

void PlanetSubsystem::stepUpdateGraphicsAll(const glm::dvec3& cameraPos) {
    for (const std::shared_ptr<Planet>& planet : m_planets) {
        planet->updateGraphics(cameraPos);
    }
}

size_t PlanetSubsystem::computeHash() const {
    size_t hash{0};
    for (const std::shared_ptr<Planet>& planet : m_planets) {
        hash = Hash::combineHashes(hash, planet->computeHash());
    }
    return hash;
}
