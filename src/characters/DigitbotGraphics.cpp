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

// Direction should already be normalized.
glm::dquat quatLookAtYForward(const glm::dvec3& direction, const glm::dvec3& up) {
    // Normalize direction
    //glm::dvec3 forward = glm::normalize(direction);
    glm::dvec3 forward = direction;
    
    // Calculate right vector (perpendicular to forward and up)
    glm::dvec3 right = glm::normalize(glm::cross(forward, up));
    
    // Recalculate up to ensure orthogonality
    glm::dvec3 upAdjusted = glm::cross(right, forward);
    
    // Build rotation matrix where:
    // - Column 0 (+X axis) points in right direction
    // - Column 1 (+Y axis) points in forward direction  
    // - Column 2 (+Z axis) points in up direction
    glm::dmat3 rotationMatrix(
        right.x, forward.x, upAdjusted.x,
        right.y, forward.y, upAdjusted.y,
        right.z, forward.z, upAdjusted.z
    );
    
    // Convert to quaternion
    return glm::quat_cast(rotationMatrix);
}

void DigitbotGraphics::updateBodyPartPositions(const DigitbotTargetPose& targetPose) {
    // Hardcoded arm measurements in local space
    const glm::dvec3 shoulderPos(0.26788, -0.044638, 1.47241);
    const glm::dvec3 naturalElbowPos(0.687425, -0.044638, 1.47241);
    const glm::dvec3 naturalHandPos(1.09582, -0.05088, 1.47241);
    
    const double upperArmLength = glm::length(naturalElbowPos - shoulderPos);  // 0.419545m
    const double lowerArmLength = glm::length(naturalHandPos - naturalElbowPos);  // ~0.4084m
    
    // Get hand target from target pose
    glm::dvec3 rightHandTarget = targetPose.rightHand.position;

    // Iterative constraint solving - converges over multiple frames
    // Constraint 1: Elbow wants to be in the lower right direction
    {
        glm::dvec3 preferredDirection = glm::normalize(glm::dvec3(1.0, 0.0, -1.0));  // +X right, -Z down
        double nudgeStrength = 0.01;
        m_rightElbowPoint += preferredDirection * nudgeStrength;
    }

    // Constraint 2: Elbow must be upperArmLength from shoulder
    {
        glm::dvec3 direction = m_rightElbowPoint - shoulderPos;
        double currentDistance = glm::length(direction);
        if (currentDistance > 1e-6) {
            m_rightElbowPoint = shoulderPos + glm::normalize(direction) * upperArmLength;
        } else {
            m_rightElbowPoint = shoulderPos + glm::dvec3(1.0, 0.0, 0.0) * upperArmLength;
        }
    }

    // Constraint 3: Elbow must be lowerArmLength from hand target
    {
        glm::dvec3 direction = m_rightElbowPoint - rightHandTarget;
        double currentDistance = glm::length(direction);
        if (currentDistance > 1e-6) {
            m_rightElbowPoint = rightHandTarget + glm::normalize(direction) * lowerArmLength;
        } else {
            m_rightElbowPoint = rightHandTarget + glm::dvec3(1.0, 0.0, 0.0) * lowerArmLength;
        }
    }

    // Constraint 3: Elbow wants to be in the lower right direction.

    { // Right upper arm.
        // Calculate upper arm orientation to point from shoulder to elbow
        glm::dvec3 direction = glm::normalize(m_rightElbowPoint - shoulderPos);
        glm::dvec3 up = glm::normalize(glm::cross(
            rightHandTarget - m_rightElbowPoint,
            shoulderPos - m_rightElbowPoint
        ));
        //glm::dvec3 up(0.0, 0.0, 1.0);  // +Z is up in this coordinate system
        
        // quatLookAt makes -Z point along direction (camera convention)
        glm::dquat lookAtQuat = quatLookAtYForward(direction, up);
        
        // Invert because lookAt is for cameras (inverse of object orientation)
        glm::dquat invertedLookAt = glm::inverse(lookAtQuat);
        // Need to correct as arm points to the +X not +Y
        glm::dquat correction = glm::angleAxis(glm::radians(90.), glm::dvec3(0.0, 0.0, 1.0));
        
        glm::dquat finalOrientation = invertedLookAt * correction;

        // Update instance - rotate around pivot
        {
            auto geometry = m_bodyPartGeometries[RIGHT_UPPER_ARM].lock();
            auto instance = m_bodyPartInstances[RIGHT_UPPER_ARM].lock();
            
            if (geometry && instance) {
                instance->m_localOrientation = finalOrientation;
                glm::dvec3 rotationShift =
                    finalOrientation * shoulderPos - shoulderPos;
                instance->m_localPosition = -rotationShift;

                geometry->updateInstanceInBuffer(instance.get());
            }
        }
    }

    { // Right lower arm.
        // Calculate upper arm orientation to point from shoulder to elbow
        glm::dvec3 direction = glm::normalize(rightHandTarget - m_rightElbowPoint);
        glm::dvec3 up = glm::normalize(glm::cross(
            rightHandTarget - m_rightElbowPoint,
            shoulderPos - m_rightElbowPoint
        ));
        //glm::dvec3 up(0.0, 0.0, 1.0);  // +Z is up in this coordinate system
        
        // quatLookAt makes -Z point along direction (camera convention)
        glm::dquat lookAtQuat = quatLookAtYForward(direction, up);
        
        // Invert because lookAt is for cameras (inverse of object orientation)
        glm::dquat invertedLookAt = glm::inverse(lookAtQuat);
        // Need to correct as arm points to the +X not +Y
        glm::dquat correction = glm::angleAxis(glm::radians(90.), glm::dvec3(0.0, 0.0, 1.0));
        
        glm::dquat finalOrientation = invertedLookAt * correction;

        // Update instance - rotate around pivot
        {
            auto geometry = m_bodyPartGeometries[RIGHT_LOWER_ARM].lock();
            auto instance = m_bodyPartInstances[RIGHT_LOWER_ARM].lock();
            
            if (geometry && instance) {
                instance->m_localOrientation = finalOrientation;
                glm::dvec3 rotationShift =
                    finalOrientation * naturalElbowPos - naturalElbowPos;
                glm::dvec3 elbowShift = m_rightElbowPoint - naturalElbowPos;
                instance->m_localPosition = -rotationShift + elbowShift;

                geometry->updateInstanceInBuffer(instance.get());
            }
        }
    }
}