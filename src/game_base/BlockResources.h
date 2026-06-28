// BlockResources.h
#pragma once

#include <memory>
#include <optional>
#include <string>

class GraphicsEngine;
class Geometry;

// Shared geometry and textures for one type of special block.
// One instance per block type per Grid, lazily created on first placement.
// maskTexturePath is optional — pass std::nullopt for blocks that don't use a mask.
class BlockResources {
public:
    BlockResources(GraphicsEngine* graphics,
                   const std::string& geometryPath,
                   const std::string& colorTexturePath,
                   const std::string& normalTexturePath,
                   const std::optional<std::string>& maskTexturePath = std::nullopt);
    ~BlockResources();

    BlockResources(const BlockResources&) = delete;
    BlockResources& operator=(const BlockResources&) = delete;

    std::weak_ptr<Geometry> getGeometry()    const { return m_geometry; }
    int getColorTextureUnit()                const { return m_colorTextureUnit; }
    int getNormalTextureUnit()               const { return m_normalTextureUnit; }
    int getMaskTextureUnit()                 const { return m_maskTextureUnit; }  // -1 if unused

private:
    GraphicsEngine*         m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    int m_colorTextureUnit{-1};
    int m_normalTextureUnit{-1};
    int m_maskTextureUnit{-1};
};
