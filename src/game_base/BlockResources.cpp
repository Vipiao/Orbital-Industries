// BlockResources.cpp
#include "BlockResources.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <cassert>
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
        assert(part.alpha > 0.0 && part.alpha <= 1.0 &&
               "block part alpha must be in (0, 1]");

        std::weak_ptr<Geometry> geometry = m_graphics->createInstanceGeometry(
            part.geometryPath,
            part.alpha < 1.0 ? RenderLayer::Transparent : RenderLayer::Opaque);
        if (geometry.expired()) {
            releaseGeometries(); // unwind the parts already loaded before failing
            throw std::runtime_error{"BlockResources: failed to load geometry from "
                                     + part.geometryPath};
        }
        m_parts.push_back(Part{geometry, part.alpha});
    }
}

void BlockResources::releaseGeometries() {
    for (const Part& part : m_parts) {
        if (!part.geometry.expired()) {
            m_graphics->releaseInstanceGeometry(part.geometry);
        }
    }
    m_parts.clear();
}

BlockResources::~BlockResources() {
    releaseGeometries();
}
