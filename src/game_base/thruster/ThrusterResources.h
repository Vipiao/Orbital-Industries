// ThrusterResources.h
#pragma once

#include <memory>

class GraphicsEngine;
class Geometry;

/**
 * @brief Shared geometry and textures for all thruster instances.
 * One per Grid (lazily created on first addThruster).
 */
class ThrusterResources {
public:
    explicit ThrusterResources(GraphicsEngine* graphics);
    ~ThrusterResources();

    ThrusterResources(const ThrusterResources&) = delete;
    ThrusterResources& operator=(const ThrusterResources&) = delete;

    std::weak_ptr<Geometry> getGeometry() const { return m_geometry; }
    int getColorTextureUnit() const { return m_colorTextureUnit; }
    int getNormalTextureUnit() const { return m_normalTextureUnit; }

private:
    GraphicsEngine* m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    int m_colorTextureUnit{-1};
    int m_normalTextureUnit{-1};

    void loadResources();
};
