// DigitbotGraphics.cpp
#include "DigitbotGraphics.h"
#include "DigitbotResources.h"
#include "ArticulationUtils.h"
#include "../graphics/GraphicsEngine.h"
#include "../graphics/instanceHandler/InstanceHandler.h"
#include "../graphics/SSBOManager.h"
#include <iostream>

DigitbotGraphics::DigitbotGraphics(GraphicsEngine* graphics, DigitbotResources* resources)
    : m_graphics(graphics)
    , m_resources(resources)
    , m_visualMeshSSBOIndex(-1)
    , m_leftElbowPoint(0.0, 0.0, 0.0)
    , m_rightElbowPoint(0.0, 0.0, 0.0)
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

        //instance->m_localPosition = glm::dvec3(0.5, 0.5, 0.0);
        instance->m_localPosition = glm::dvec3(0.0, 0.0, 0.0);
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
    // Arrays for both arms (right=0, left=1)
    glm::dvec3* elbows[2] = {&m_rightElbowPoint, &m_leftElbowPoint};
    BodyPart upperArms[2] = {RIGHT_UPPER_ARM, LEFT_UPPER_ARM};
    BodyPart lowerArms[2] = {RIGHT_LOWER_ARM, LEFT_LOWER_ARM};
    glm::dvec3 shoulders[2] = {m_resources->m_naturalRightShoulderPos, m_resources->m_naturalLeftShoulderPos};
    glm::dvec3 naturalElbows[2] = {m_resources->m_naturalRightElbowPos, m_resources->m_naturalLeftElbowPos};
    glm::dvec3 handTargets[2] = {targetPose.rightHand.position, targetPose.leftHand.position};
    glm::dvec3 preferredDirections[2] = {
        glm::normalize(glm::dvec3(1.0, 0.0, -1.0)),   // Right: +X right, -Z down
        glm::normalize(glm::dvec3(-1.0, 0.0, -1.0))   // Left: -X left, -Z down
    };
    
    // Model-specific correction: arms point along +X in model space, but we calculate along +Y
    glm::dquat rightCorrectionQuat = glm::angleAxis(glm::radians(90.0), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat leftCorrectionQuat = glm::conjugate(rightCorrectionQuat);
    glm::dquat correctionQuats[2] = {rightCorrectionQuat, leftCorrectionQuat};
   
    // First loop: Iterative constraint solving for both arms
    for (int ii = 0; ii < 2; ++ii) {
        // Constraint 1: Elbow wants to be in the lower right/left direction
        *elbows[ii] = ArticulationUtils::applyDirectionNudge(*elbows[ii], preferredDirections[ii], 0.01);

        // Constraint 2: Elbow must be upperArmLength from shoulder
        *elbows[ii] = ArticulationUtils::applyDistanceConstraint(*elbows[ii], shoulders[ii], m_resources->m_upperArmLength);

        // Constraint 3: Elbow must be lowerArmLength from hand target
        *elbows[ii] = ArticulationUtils::applyDistanceConstraint(*elbows[ii], handTargets[ii], m_resources->m_lowerArmLength);
    }
    
    // Second loop: Articulation for both arms
    for (int ii = 0; ii < 2; ++ii) {
        // Calculate up vector for arm orientation (flip sign for left arm due to mirroring)
        double sign = (ii == 0 ? 1.0 : -1.0);
        glm::dvec3 up = sign * glm::normalize(glm::cross(
            handTargets[ii] - *elbows[ii],
            shoulders[ii] - *elbows[ii]
        ));
        
        // Update upper arm
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[upperArms[ii]],
            m_bodyPartInstances[upperArms[ii]],
            shoulders[ii],      // directionFrom
            *elbows[ii],        // directionTo
            up,                 // upVector
            correctionQuats[ii],     // correctionQuat
            shoulders[ii]       // naturalPivot
        );

        // Update lower arm
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[lowerArms[ii]],
            m_bodyPartInstances[lowerArms[ii]],
            *elbows[ii],        // directionFrom
            handTargets[ii],    // directionTo
            up,                 // upVector
            correctionQuats[ii],     // correctionQuat
            naturalElbows[ii]   // naturalPivot
        );
    }
}