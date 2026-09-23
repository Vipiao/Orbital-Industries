// PlanetSubsystem.cpp
#include "PlanetSubsystem.h"
#include "Planet.h"
#include "PlanetType.h"
#include "utils/HashFunctions.h"
#include <stdexcept>

PlanetSubsystem::PlanetSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics)
    : m_physics{physics}, m_graphics{graphics} {
    if (!m_physics || !m_graphics) {
        throw std::runtime_error("PlanetSubsystem: All dependencies must be non-null");
    }
}

PlanetSubsystem::~PlanetSubsystem() = default;

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
    m_planets.push_back(std::make_shared<Planet>(m_physics, m_graphics, *lockedType, massKg));
    return m_planets.back();
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
