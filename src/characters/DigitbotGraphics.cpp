// DigitbotGraphics.cpp
#include "DigitbotGraphics.h"
#include "DigitbotResources.h"
#include "../graphics/GraphicsEngine.h"
#include "../graphics/instanceHandler/InstanceHandler.h"
#include "../graphics/SSBOManager.h"
#include <iostream>

DigitbotGraphics::DigitbotGraphics(GraphicsEngine* graphics, DigitbotResources* resources)
    : m_graphics(graphics)
    , m_resources(resources)
    , m_visualMeshSSBOIndex(-1)
{
    if (!m_resources) {
        throw std::runtime_error("DigitbotResources cannot be null");
    }

    // Allocate SSBO index for shared world transform
    m_visualMeshSSBOIndex = m_graphics->m_ssboManager->allocateIndex();

    // Copy shared geometries from resources
    m_bodyPartGeometries = m_resources->getBodyPartGeometries();

    // Reserve space for instances
    m_bodyPartInstances.resize(PART_COUNT);

    // Create instances for all body parts using shared geometries
    for (int i = 0; i < PART_COUNT; ++i) {
        auto geometry = m_bodyPartGeometries[i].lock();
        if (!geometry) {
            throw std::runtime_error("DigitbotGraphics: Shared geometry is invalid for body part " + std::to_string(i));
        }
        
        // Create instance attached to shared SSBO slot
        m_bodyPartInstances[i] = geometry->addInstance(
            m_visualMeshSSBOIndex,              // meshIndex - shared SSBO slot
            m_resources->getColorTextureUnit(), // colorTextureUnit
            m_resources->getNormalTextureUnit(),// normalTextureUnit
            -1,                                  // materialTextureUnit - no material
            glm::dvec4(1.0, 0.0, 0.0, 1.0));    // white color
    }
    
    // Initialize all instance transforms
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