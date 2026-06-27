// ThrusterGraphics.cpp
#include "ThrusterGraphics.h"
#include "ThrusterResources.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>

ThrusterGraphics::ThrusterGraphics(ThrusterResources* resources,
                                   int ssboIndex,
                                   const glm::ivec3& anchorCoord,
                                   const glm::dquat& orientation)
    : m_resources(resources)
{
    if (!m_resources) {
        throw std::runtime_error("ThrusterGraphics: ThrusterResources cannot be null");
    }

    m_geometry = m_resources->getGeometry();
    auto geometry = m_geometry.lock();
    if (!geometry) {
        throw std::runtime_error("ThrusterGraphics: geometry is invalid");
    }

    m_instance = geometry->addInstance(
        ssboIndex,
        m_resources->getColorTextureUnit(),
        m_resources->getNormalTextureUnit(),
        -1,
        glm::dvec4{1.0, 1.0, 1.0, 1.0},
        -1);

    auto instance = m_instance.lock();
    if (!instance) {
        throw std::runtime_error("ThrusterGraphics: failed to create instance");
    }

    // Rotate around the anchor cell's centre (0.5,0.5,0.5 in model space) rather
    // than the model origin, so the mesh stays aligned with the collider cells.
    static constexpr glm::dvec3 pivot{0.5, 0.5, 0.5};
    instance->m_localPosition    = glm::dvec3{anchorCoord} + pivot - glm::dvec3{orientation * pivot};
    instance->m_localOrientation = orientation;
    instance->m_localScale       = glm::dvec3{1.0, 1.0, 1.0};
    geometry->updateInstanceInBuffer(instance.get());
}

ThrusterGraphics::~ThrusterGraphics() {
    auto geometry = m_geometry.lock();
    auto instance = m_instance.lock();
    if (geometry && instance) {
        geometry->removeInstance(instance);
    }
}
