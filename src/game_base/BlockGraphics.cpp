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

    // Rotating about modelCentre keeps the block's centre fixed in the cell.
    const glm::dvec3 localPosition{
        glm::dvec3{anchorCoord} + modelCentre - glm::dvec3{orientation * modelCentre}};

    m_parts.reserve(m_resources->getParts().size());
    for (const BlockResources::Part& part : m_resources->getParts()) {
        auto geometry = part.geometry.lock();
        if (!geometry) {
            throw std::runtime_error{"BlockGraphics: geometry is invalid"};
        }

        std::weak_ptr<Instance> instanceHandle = geometry->addInstance(
            ssboIndex,
            m_resources->getColorTextureUnit(),
            m_resources->getNormalTextureUnit(),
            m_resources->getMaskTextureUnit(),
            glm::dvec4{1.0, 1.0, 1.0, part.alpha},
            -1);

        auto instance = instanceHandle.lock();
        if (!instance) {
            throw std::runtime_error{"BlockGraphics: failed to create instance"};
        }

        instance->m_localPosition    = localPosition;
        instance->m_localOrientation = orientation;
        instance->m_localScale       = glm::dvec3{1.0, 1.0, 1.0};
        geometry->updateInstanceInBuffer(instance.get());

        m_parts.push_back(PartInstance{part.geometry, instanceHandle});
    }
}

BlockGraphics::~BlockGraphics() {
    for (const PartInstance& part : m_parts) {
        auto geometry = part.geometry.lock();
        auto instance = part.instance.lock();
        if (geometry && instance) {
            geometry->removeInstance(instance);
        }
    }
}
