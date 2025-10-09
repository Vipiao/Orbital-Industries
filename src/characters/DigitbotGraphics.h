// DigitbotGraphics.h
#pragma once

#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine;
class Geometry;
class Instance;
class DigitbotResources;

/**
 * @brief Transform for a body part in local space
 */
struct BodyPartTransform {
    glm::dvec3 position;
    glm::dquat orientation;
    
    BodyPartTransform()
        : position(0.0, 0.0, 0.0)
        , orientation(1.0, 0.0, 0.0, 0.0)
    {}
    
    BodyPartTransform(const glm::dvec3& pos, const glm::dquat& ori)
        : position(pos)
        , orientation(ori)
    {}
};

/**
 * @brief Target pose for Digitbot IK solver
 */
struct DigitbotTargetPose {
    BodyPartTransform leftHand;
    BodyPartTransform rightHand;
    BodyPartTransform leftFoot;
    BodyPartTransform rightFoot;
    glm::dquat headOrientation;
    
    // TODO: Define default rest pose values
    DigitbotTargetPose()
        : leftHand()
        , rightHand()
        , leftFoot()
        , rightFoot()
        , headOrientation(1.0, 0.0, 0.0, 0.0)
    {}
};

/**
 * @brief Manages visual rendering for Digitbot character
 * 
 * Loads and updates articulated body parts using instanced rendering.
 * All body parts share a single SSBO slot for efficient rendering.
 */
class DigitbotGraphics {
public:
    DigitbotGraphics(GraphicsEngine* graphics, DigitbotResources* resources);
    ~DigitbotGraphics();

    // World transform update (called each physics step)
    void updateWorldTransform(
        const glm::dvec3& position,
        const glm::dvec3& velocity,
        const glm::dquat& orientation,
        const glm::dvec3& angVelAxis,
        double angVelMagnitude,
        const glm::dvec3& centerOfRotation,
        uint64_t physicsTimeStep);

    // Body part articulation (IK target)
    void updateBodyPartPositions(const DigitbotTargetPose& targetPose);

    int getSSBOIndex() const { return m_visualMeshSSBOIndex; }

private:
    enum BodyPart {
        LEFT_FOOT = 0,
        LEFT_LOWER_ARM = 1,
        LEFT_LOWER_LEG = 2,
        LEFT_PISTON_HOUSING = 3,
        LEFT_PISTON_ROD = 4,
        LEFT_UPPER_ARM = 5,
        LEFT_UPPER_LEG = 6,
        RIGHT_FOOT = 7,
        RIGHT_LOWER_ARM = 8,
        RIGHT_LOWER_LEG = 9,
        RIGHT_PISTON_HOUSING = 10,
        RIGHT_PISTON_ROD = 11,
        RIGHT_UPPER_ARM = 12,
        RIGHT_UPPER_LEG = 13,
        HEAD = 14,
        BODY = 15,
        PART_COUNT = 16
    };

    void initializeInstanceTransforms();

    GraphicsEngine* m_graphics;
    DigitbotResources* m_resources;  // Non-owning pointer to shared resources
    int m_visualMeshSSBOIndex;

    // One geometry and instance per body part
    std::vector<std::weak_ptr<Geometry>> m_bodyPartGeometries;
    std::vector<std::weak_ptr<Instance>> m_bodyPartInstances;

    // IK solver state
    glm::dvec3 m_rightElbowPoint;
};