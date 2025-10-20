// DigibotResources.cpp
#include "DigibotResources.h"
#include "../../graphics/GraphicsEngine.h"
#include "../../graphics/instanceHandler/InstanceHandler.h"
#include <iostream>

DigibotResources::DigibotResources(GraphicsEngine* graphics)
    : m_graphics(graphics)
    , m_colorTextureUnit(-1)
    , m_normalTextureUnit(-1)
{
    if (!m_graphics) {
        throw std::runtime_error("GraphicsEngine cannot be null");
    }
    
    loadResources();
}

DigibotResources::~DigibotResources() {
    // Release all geometries
    for (auto& geometryWeak : m_bodyPartGeometries) {
        if (!geometryWeak.expired()) {
            m_graphics->getInstanceHandler()->releaseGeometry(geometryWeak);
        }
    }
    
    // Textures are automatically cleaned up by TextureManager destructor
}

void DigibotResources::loadResources() {
    // Load shared texture atlas
    m_colorTextureUnit = m_graphics->getInstanceHandler()->createTexture("../media/textures/robot/atlas.png");
    m_normalTextureUnit = m_graphics->getInstanceHandler()->createTexture("../media/textures/robot/atlas_normal.png");
    
    if (m_colorTextureUnit == -1 || m_normalTextureUnit == -1) {
        throw std::runtime_error("Failed to load Digibot textures");
    }
    
    std::cout << "DigibotResources: Loaded textures - Color unit: " << m_colorTextureUnit 
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
            throw std::runtime_error("Failed to load Digibot geometry: " + std::string(filePaths[i]));
        }
        m_bodyPartGeometries.push_back(geometry);
    }
    
    std::cout << "DigibotResources: Loaded " << m_bodyPartGeometries.size() << " body part geometries" << std::endl;
}