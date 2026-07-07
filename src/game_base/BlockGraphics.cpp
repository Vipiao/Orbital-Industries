// BlockGraphics.cpp
#include "BlockGraphics.h"
#include "BlockResources.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>

BlockGraphics::BlockGraphics(BlockResources* resources,
                             int ssboIndex,
                             const glm::ivec3& anchorCoord,
                             const glm::dquat& orientation,
                             const glm::dvec3& modelCentre)
    : m_resources{resources}
{
    if (!m_resources) {
        throw std::runtime_error{"BlockGraphics: BlockResources cannot be null"};
    }

    m_geometry = m_resources->getGeometry();
    auto geometry = m_geometry.lock();
    if (!geometry) {
        throw std::runtime_error{"BlockGraphics: geometry is invalid"};
    }

    m_instance = geometry->addInstance(
        ssboIndex,
        m_resources->getColorTextureUnit(),
        m_resources->getNormalTextureUnit(),
        m_resources->getMaskTextureUnit(),
        glm::dvec4{1.0, 1.0, 1.0, m_resources->getAlpha()},
        -1);

    auto instance = m_instance.lock();
    if (!instance) {
        throw std::runtime_error{"BlockGraphics: failed to create instance"};
    }

    instance->m_localPosition    = glm::dvec3{anchorCoord} + modelCentre - glm::dvec3{orientation * modelCentre};
    instance->m_localOrientation = orientation;
    instance->m_localScale       = glm::dvec3{1.0, 1.0, 1.0};
    geometry->updateInstanceInBuffer(instance.get());
}

BlockGraphics::~BlockGraphics() {
    auto geometry = m_geometry.lock();
    auto instance = m_instance.lock();
    if (geometry && instance) {
        geometry->removeInstance(instance);
    }
}
