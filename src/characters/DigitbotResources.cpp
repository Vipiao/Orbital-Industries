// DigitbotResources.cpp
#include "DigitbotResources.h"
#include "../graphics/GraphicsEngine.h"
#include "../graphics/instanceHandler/InstanceHandler.h"
#include <iostream>

DigitbotResources::DigitbotResources(GraphicsEngine* graphics)
    : m_graphics(graphics)
    , m_colorTextureUnit(-1)
    , m_normalTextureUnit(-1)
{
    if (!m_graphics) {
        throw std::runtime_error("GraphicsEngine cannot be null");
    }
    
    loadResources();
    initializeSkeletonMeasurements();
}

DigitbotResources::~DigitbotResources() {
    // Release all geometries
    for (auto& geometryWeak : m_bodyPartGeometries) {
        if (!geometryWeak.expired()) {
            m_graphics->getInstanceHandler()->releaseGeometry(geometryWeak);
        }
    }
    
    // Textures are automatically cleaned up by TextureManager destructor
}

void DigitbotResources::initializeSkeletonMeasurements() {
    // Right arm measurements in local space (T-pose)
    m_naturalRightShoulderPos = glm::dvec3(0.26788, -0.044638, 1.47241);
    m_naturalRightElbowPos = glm::dvec3(0.687425, -0.044638, 1.47241);
    m_naturalRightHandPos = glm::dvec3(1.09582, -0.05088, 1.47241);
    
    // Left arm measurements (mirrored X coordinates)
    m_naturalLeftShoulderPos = glm::dvec3(-0.26788, -0.044638, 1.47241);
    m_naturalLeftElbowPos = glm::dvec3(-0.687425, -0.044638, 1.47241);
    m_naturalLeftHandPos = glm::dvec3(-1.09582, -0.05088, 1.47241);
    
    // Limb lengths
    m_upperArmLength = glm::length(m_naturalRightElbowPos - m_naturalRightShoulderPos);
    m_lowerArmLength = glm::length(m_naturalRightHandPos - m_naturalRightElbowPos);
}

void DigitbotResources::loadResources() {
    // Load shared texture atlas
    m_colorTextureUnit = m_graphics->getInstanceHandler()->createTexture("../media/textures/robot/atlas.png");
    m_normalTextureUnit = m_graphics->getInstanceHandler()->createTexture("../media/textures/robot/atlas_normal.png");
    
    if (m_colorTextureUnit == -1 || m_normalTextureUnit == -1) {
        throw std::runtime_error("Failed to load Digitbot textures");
    }
    
    std::cout << "DigitbotResources: Loaded textures - Color unit: " << m_colorTextureUnit 
              << ", Normal unit: " << m_normalTextureUnit << std::endl;
    
    // File paths for all 16 body parts
    const char* filePaths[16] = {
        "../media/characters/left_foot.obj",
        "../media/characters/left_lower_arm.obj",
        "../media/characters/left_lower_leg.obj",
        "../media/characters/left_piston_housing.obj",
        "../media/characters/left_piston_rod.obj",
        "../media/characters/left_upper_arm.obj",
        "../media/characters/left_upper_leg.obj",
        "../media/characters/right_foot.obj",
        "../media/characters/right_lower_arm.obj",
        "../media/characters/right_lower_leg.obj",
        "../media/characters/right_piston_housing.obj",
        "../media/characters/right_piston_rod.obj",
        "../media/characters/right_upper_arm.obj",
        "../media/characters/right_upper_leg.obj",
        "../media/characters/head.obj",
        "../media/characters/body.obj"
    };
    
    // Reserve space and load all geometries
    m_bodyPartGeometries.reserve(16);
    
    for (int i = 0; i < 16; ++i) {
        auto geometry = m_graphics->getInstanceHandler()->createGeometry(filePaths[i]);
        if (geometry.expired()) {
            throw std::runtime_error("Failed to load Digitbot geometry: " + std::string(filePaths[i]));
        }
        m_bodyPartGeometries.push_back(geometry);
    }
    
    std::cout << "DigitbotResources: Loaded " << m_bodyPartGeometries.size() << " body part geometries" << std::endl;
}