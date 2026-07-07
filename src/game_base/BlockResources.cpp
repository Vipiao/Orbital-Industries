// BlockResources.cpp
#include "BlockResources.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>

BlockResources::BlockResources(GraphicsEngine* graphics,
                               const std::vector<BlockGeometryPart>& parts,
                               const std::string& colorTexturePath,
                               const std::string& normalTexturePath,
                               const std::optional<std::string>& maskTexturePath)
    : m_graphics{graphics}
{
    if (!m_graphics) {
        throw std::runtime_error{"BlockResources: GraphicsEngine cannot be null"};
    }
    if (parts.empty()) {
        throw std::runtime_error{"BlockResources: block must have at least one part"};
    }

    m_colorTextureUnit  = m_graphics->createInstanceTexture(colorTexturePath);
    m_normalTextureUnit = m_graphics->createInstanceTexture(normalTexturePath);

    if (m_colorTextureUnit == -1 || m_normalTextureUnit == -1) {
        throw std::runtime_error{"BlockResources: failed to load textures from " + colorTexturePath};
    }

    if (maskTexturePath) {
        m_maskTextureUnit = m_graphics->createInstanceTexture(*maskTexturePath);
    }

    m_parts.reserve(parts.size());
    for (const BlockGeometryPart& part : parts) {
        std::weak_ptr<Geometry> geometry = m_graphics->createInstanceGeometry(
            part.geometryPath, /*transparent=*/part.alpha < 1.0);
        if (geometry.expired()) {
            throw std::runtime_error{"BlockResources: failed to load geometry from "
                                     + part.geometryPath};
        }
        m_parts.push_back(Part{geometry, part.alpha});
    }
}

BlockResources::~BlockResources() {
    for (const Part& part : m_parts) {
        if (!part.geometry.expired()) {
            m_graphics->releaseInstanceGeometry(part.geometry);
        }
    }
}
