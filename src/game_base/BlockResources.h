// BlockResources.h
#pragma once

#include "BlockGeometryPart.h"
#include <memory>
#include <optional>
#include <string>
#include <vector>

class GraphicsEngine;
class Geometry;

// Shared textures and geometry for one type of special block.
// A block is made of one or more parts (frame, doors, glass, ...). Every part
// shares this block's colour/normal/mask textures but owns its own geometry and
// opacity, so opaque and translucent parts can coexist in a single block.
// maskTexturePath is optional — pass std::nullopt for blocks that don't use one.
class BlockResources {
public:
    // A loaded part: the shared geometry handle plus the opacity every instance
    // of it is tinted with. alpha < 1.0 means the geometry lives in the OIT pass.
    struct Part {
        std::weak_ptr<Geometry> geometry;
        double                  alpha{1.0};
    };

    BlockResources(GraphicsEngine* graphics,
                   const std::vector<BlockGeometryPart>& parts,
                   const std::string& colorTexturePath,
                   const std::string& normalTexturePath,
                   const std::optional<std::string>& maskTexturePath = std::nullopt);
    ~BlockResources();

    BlockResources(const BlockResources&) = delete;
    BlockResources& operator=(const BlockResources&) = delete;

    const std::vector<Part>& getParts() const { return m_parts; }
    int getColorTextureUnit()            const { return m_colorTextureUnit; }
    int getNormalTextureUnit()           const { return m_normalTextureUnit; }
    int getMaskTextureUnit()             const { return m_maskTextureUnit; }  // -1 if unused

private:
    GraphicsEngine*   m_graphics;
    std::vector<Part> m_parts;
    int m_colorTextureUnit{-1};
    int m_normalTextureUnit{-1};
    int m_maskTextureUnit{-1};
};
