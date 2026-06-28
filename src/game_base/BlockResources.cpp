// BlockResources.cpp
#include "BlockResources.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>

BlockResources::BlockResources(GraphicsEngine* graphics,
                               const std::string& geometryPath,
                               const std::string& colorTexturePath,
                               const std::string& normalTexturePath,
                               const std::optional<std::string>& maskTexturePath)
    : m_graphics{graphics}
{
    if (!m_graphics) {
        throw std::runtime_error{"BlockResources: GraphicsEngine cannot be null"};
    }

    m_colorTextureUnit  = m_graphics->getInstanceHandler()->createTexture(colorTexturePath);
    m_normalTextureUnit = m_graphics->getInstanceHandler()->createTexture(normalTexturePath);

    if (m_colorTextureUnit == -1 || m_normalTextureUnit == -1) {
        throw std::runtime_error{"BlockResources: failed to load textures from " + colorTexturePath};
    }

    if (maskTexturePath) {
        m_maskTextureUnit = m_graphics->getInstanceHandler()->createTexture(*maskTexturePath);
    }

    m_geometry = m_graphics->getInstanceHandler()->createGeometry(geometryPath);
    if (m_geometry.expired()) {
        throw std::runtime_error{"BlockResources: failed to load geometry from " + geometryPath};
    }
}

BlockResources::~BlockResources() {
    if (!m_geometry.expired()) {
        m_graphics->getInstanceHandler()->releaseGeometry(m_geometry);
    }
}
