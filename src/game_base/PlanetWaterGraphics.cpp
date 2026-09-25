// PlanetWaterGraphics.cpp
#include "PlanetWaterGraphics.h"
#include "PlanetWaterConfig.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"  // Geometry / Instance
#include <cassert>
#include <stdexcept>

namespace {
// The shell mesh is a unit icosphere whose faces come within 0.982 of its centre,
// so this puts every face outside the sea-level sphere
constexpr double k_shellMargin{1.02};
}  // namespace

PlanetWaterGraphics::PlanetWaterGraphics(GraphicsEngine* graphics,
                                         std::weak_ptr<Geometry> shellGeometry,
                                         int ssboIndex, double seaLevelRadius,
                                         const PlanetWaterConfig& config)
    : m_graphics{graphics}, m_geometry{shellGeometry} {
    if (!m_graphics) {
        throw std::runtime_error{"PlanetWaterGraphics: GraphicsEngine cannot be null"};
    }
    assert(glm::all(glm::greaterThan(config.m_absorptionPerMetre, glm::dvec3{0.0})) &&
           "water must absorb in every channel to be seen");

    assert(seaLevelRadius > 0.0);
    // The shader reads it as a float; exact, it matches what CPU code expects
    assert(static_cast<double>(static_cast<float>(seaLevelRadius)) == seaLevelRadius &&
           "sea-level radius must be exact in a float");

    // state.xyz is the absorption per channel and state.w the sea-level radius;
    // neither changes
    m_instance = m_graphics->addRayVolumeInstance(
        m_geometry, ssboIndex, glm::dvec4{config.m_color, 1.0},
        glm::dvec4{config.m_absorptionPerMetre, seaLevelRadius});

    std::shared_ptr<Instance> instance{m_instance.lock()};
    std::shared_ptr<Geometry> geometry{m_geometry.lock()};
    if (!instance || !geometry) {
        throw std::runtime_error{"PlanetWaterGraphics: failed to create water instance"};
    }

    // Centred on the planet's local origin, like the surface
    instance->m_localPosition = glm::dvec3{0.0};
    instance->m_localOrientation = glm::dquat{1.0, 0.0, 0.0, 0.0};
    instance->m_localScale = glm::dvec3{seaLevelRadius * k_shellMargin};
    geometry->updateInstanceInBuffer(instance.get());
}

PlanetWaterGraphics::~PlanetWaterGraphics() {
    m_graphics->removeRayVolumeInstance(m_geometry, m_instance);
}
