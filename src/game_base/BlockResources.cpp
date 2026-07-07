// BlockResources.cpp
#include "BlockResources.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>

BlockResources::BlockResources(GraphicsEngine* graphics,
                               const std::string& geometryPath,
                               const std::string& colorTexturePath,
                               const std::string& normalTexturePath,
                               const std::optional<std::string>& maskTexturePath,
                               double alpha)
    : m_graphics{graphics}, m_alpha{alpha}
{
    if (!m_graphics) {
        throw std::runtime_error{"BlockResources: GraphicsEngine cannot be null"};
    }

    m_colorTextureUnit  = m_graphics->createInstanceTexture(colorTexturePath);
    m_normalTextureUnit = m_graphics->createInstanceTexture(normalTexturePath);

    if (m_colorTextureUnit == -1 || m_normalTextureUnit == -1) {
        throw std::runtime_error{"BlockResources: failed to load textures from " + colorTexturePath};
    }

    if (maskTexturePath) {
        m_maskTextureUnit = m_graphics->createInstanceTexture(*maskTexturePath);
    }

    m_geometry = m_graphics->createInstanceGeometry(
        geometryPath, /*transparent=*/m_alpha < 1.0);
    if (m_geometry.expired()) {
        throw std::runtime_error{"BlockResources: failed to load geometry from " + geometryPath};
    }
}

BlockResources::~BlockResources() {
    if (!m_geometry.expired()) {
        m_graphics->releaseInstanceGeometry(m_geometry);
    }
}
