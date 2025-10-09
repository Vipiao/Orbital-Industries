// DigitbotResources.h
#pragma once

#include <memory>
#include <vector>

class GraphicsEngine;
class Geometry;

/**
 * @brief Shared graphics resources for all Digitbot characters
 * 
 * Loads and manages textures and geometries that are shared across
 * all Digitbot instances to avoid redundant loading.
 */
class DigitbotResources {
public:
    explicit DigitbotResources(GraphicsEngine* graphics);
    ~DigitbotResources();

    // Delete copy/move to ensure single ownership
    DigitbotResources(const DigitbotResources&) = delete;
    DigitbotResources& operator=(const DigitbotResources&) = delete;
    DigitbotResources(DigitbotResources&&) = delete;
    DigitbotResources& operator=(DigitbotResources&&) = delete;

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