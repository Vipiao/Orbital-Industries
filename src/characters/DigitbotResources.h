// DigitbotResources.h
#pragma once

#include <memory>
#include <vector>
#include <glm/glm.hpp>

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

    // Skeleton measurements (T-pose)
    glm::dvec3 m_naturalRightShoulderPos;
    glm::dvec3 m_naturalRightElbowPos;
    glm::dvec3 m_naturalRightHandPos;
    glm::dvec3 m_naturalLeftShoulderPos;
    glm::dvec3 m_naturalLeftElbowPos;
    glm::dvec3 m_naturalLeftHandPos;
    
    double m_upperArmLength;
    double m_lowerArmLength;

private:
    GraphicsEngine* m_graphics;
    
    // Shared resources
    std::vector<std::weak_ptr<Geometry>> m_bodyPartGeometries;  // 16 body parts
    int m_colorTextureUnit;
    int m_normalTextureUnit;
    
    void loadResources();
    void initializeSkeletonMeasurements();
};