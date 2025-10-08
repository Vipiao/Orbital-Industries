// DigitbotGraphics.cpp
#include "DigitbotGraphics.h"
#include "../graphics/GraphicsEngine.h"
#include "../graphics/instanceHandler/InstanceHandler.h"
#include "../graphics/SSBOManager.h"
#include <iostream>

DigitbotGraphics::DigitbotGraphics(GraphicsEngine* graphics)
    : m_graphics(graphics)
    , m_visualMeshSSBOIndex(-1)
{
    // Allocate SSBO index for shared world transform
    m_visualMeshSSBOIndex = m_graphics->m_ssboManager->allocateIndex();

    // Reserve space for all body parts
    m_bodyPartGeometries.resize(PART_COUNT);
    m_bodyPartInstances.resize(PART_COUNT);

    // Load all body part geometries and create instances
    loadBodyParts();

    // Initialize all instances to identity transforms
    initializeInstanceTransforms();
}

DigitbotGraphics::~DigitbotGraphics() {
    // Remove all instances from their geometries
    for (size_t i = 0; i < PART_COUNT; ++i) {
        auto geometry = m_bodyPartGeometries[i].lock();
        auto instance = m_bodyPartInstances[i].lock();
        if (geometry && instance) {
            geometry->removeInstance(instance);
        }
    }

    // Deallocate SSBO index
    if (m_visualMeshSSBOIndex != -1) {
        m_graphics->m_ssboManager->deallocateIndex(m_visualMeshSSBOIndex);
    }
}

void DigitbotGraphics::loadBodyParts() {
    // File paths for all body parts
    const char* filePaths[PART_COUNT] = {
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

    // Load each body part and create an instance
    for (int i = 0; i < PART_COUNT; ++i) {
        // Load geometry
        m_bodyPartGeometries[i] = m_graphics->getInstanceHandler()->createGeometry(filePaths[i]);
        
        auto geometry = m_bodyPartGeometries[i].lock();
        if (!geometry) {
            std::cerr << "DigitbotGraphics: Failed to load geometry: " << filePaths[i] << std::endl;
            continue;
        }

        // Create instance attached to shared SSBO slot  
        // Note: All body parts will share offset (0.5, 0.5, 0.0) to center them
        // over the physics cubes (model origin is at feet, cubes at corner)
        // This offset is applied in initializeInstanceTransforms()
        m_bodyPartInstances[i] = geometry->addInstance(
            m_visualMeshSSBOIndex,  // meshIndex - shared SSBO slot
            -1,                      // colorTextureUnit - no texture
            -1,                      // normalTextureUnit - no texture
            -1,                      // materialTextureUnit - no texture
            glm::dvec4(1.0, 1.0, 1.0, 1.0));  // white color
    }
}

void DigitbotGraphics::initializeInstanceTransforms() {
    // Initialize all body parts with offset to align with physics cubes
    // Model origin is at feet, cubes have origin at corner, so offset by (0.5, 0.5, 0.0)
    // to center the visual model over the cube
    for (int i = 0; i < PART_COUNT; ++i) {
        auto geometry = m_bodyPartGeometries[i].lock();
        auto instance = m_bodyPartInstances[i].lock();
        
        if (!geometry || !instance) {
            continue;
        }

        instance->m_localPosition = glm::dvec3(0.5, 0.5, 0.0);
        instance->m_localOrientation = glm::dquat(1.0, 0.0, 0.0, 0.0);
        instance->m_localScale = glm::dvec3(1.0, 1.0, 1.0);
        
        geometry->updateInstanceInBuffer(instance.get());
    }
}

void DigitbotGraphics::updateWorldTransform(
    const glm::dvec3& position,
    const glm::dvec3& velocity,
    const glm::dquat& orientation,
    const glm::dvec3& angVelAxis,
    double angVelMagnitude,
    const glm::dvec3& centerOfRotation,
    uint64_t physicsTimeStep)
{
    if (m_visualMeshSSBOIndex == -1) {
        return;
    }

    // Update shared SSBO slot with rigid body transform
    m_graphics->m_ssboManager->updateMeshTransform(
        m_visualMeshSSBOIndex,
        position,
        velocity,
        orientation,
        angVelAxis,
        angVelMagnitude,
        centerOfRotation,
        glm::dvec3(1.0, 1.0, 1.0),  // scale
        -1, -1, -1,                  // no textures
        physicsTimeStep,
        0.0                          // emissiveScalar
    );
}

void DigitbotGraphics::updateBodyPartPositions(const DigitbotTargetPose& targetPose) {
    // TODO: Implement inverse kinematics to position body parts
    // For now, all body parts remain at identity transforms (rest pose)
    // 
    // Future implementation will:
    // 1. Solve IK for limbs to reach targetPose.leftHand, rightHand, leftFoot, rightFoot
    // 2. Set head orientation from targetPose.headOrientation
    // 3. Update each instance's m_localPosition and m_localOrientation
    // 4. Call geometry->updateInstanceInBuffer() for modified instances
}