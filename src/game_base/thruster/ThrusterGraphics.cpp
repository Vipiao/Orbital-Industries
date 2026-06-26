// ThrusterGraphics.cpp
#include "ThrusterGraphics.h"
#include "ThrusterResources.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>

ThrusterGraphics::ThrusterGraphics(ThrusterResources* resources, int ssboIndex, const glm::ivec3& anchorCoord)
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

    // OBJ spans (0,0,0)→(1,2,1) — place origin at anchor coord in grid-local space
    instance->m_localPosition    = glm::dvec3{anchorCoord};
    instance->m_localOrientation = glm::dquat{1.0, 0.0, 0.0, 0.0};
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
