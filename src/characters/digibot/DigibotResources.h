// DigibotResources.h
#pragma once

#include <memory>
#include <vector>
#include <glm/glm.hpp>

class GraphicsEngine;
class Geometry;

/**
 * @brief Shared graphics resources for all Digibot characters
 * 
 * Loads and manages textures and geometries that are shared across
 * all Digibot instances to avoid redundant loading.
 */
class DigibotResources {
public:
    explicit DigibotResources(GraphicsEngine* graphics);
    ~DigibotResources();

    // Delete copy/move to ensure single ownership
    DigibotResources(const DigibotResources&) = delete;
    DigibotResources& operator=(const DigibotResources&) = delete;
    DigibotResources(DigibotResources&&) = delete;
    DigibotResources& operator=(DigibotResources&&) = delete;

    // Access to shared resources
    const std::vector<std::weak_ptr<Geometry>>& getBodyPartGeometries() const { return m_bodyPartGeometries; }
    int getColorTextureUnit() const { return m_colorTextureUnit; }
    int getNormalTextureUnit() const { return m_normalTextureUnit; }

private:
    GraphicsEngine* m_graphics;
    
    // Shared resources
    std::vector<std::weak_ptr<Geometry>> m_bodyPartGeometries;  // 16 body parts
    int m_colorTextureUnit;
    int m_normalTextureUnit;
    
    void loadResources();
};