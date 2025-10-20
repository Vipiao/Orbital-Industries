// DigibotGraphics.h
#pragma once

#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine;
class Geometry;
class Instance;
class DigibotResources;

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
 * @brief Target pose for Digibot IK solver
 */
struct DigibotTargetPose {
    BodyPartTransform leftHand;
    BodyPartTransform rightHand;
    BodyPartTransform leftFoot;
    BodyPartTransform rightFoot;
    glm::dquat headOrientation;
    
    // TODO: Define default rest pose values
    DigibotTargetPose()
        : leftHand()
        , rightHand()
        , leftFoot()
        , rightFoot()
        , headOrientation(1.0, 0.0, 0.0, 0.0)
    {}
};

/**
 * @brief Manages visual rendering for Digibot character
 * 
 * Loads and updates articulated body parts using instanced rendering.
 * All body parts share a single SSBO slot for efficient rendering.
 */
class DigibotGraphics {
public:
    DigibotGraphics(GraphicsEngine* graphics, DigibotResources* resources);
    ~DigibotGraphics();

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
    void updateBodyPartPositions(const DigibotTargetPose& targetPose);

    int getSSBOIndex() const { return m_visualMeshSSBOIndex; }

private:
    // Static skeleton measurements (T-pose natural positions)
    static const glm::dvec3 s_naturalRightShoulderPos;
    static const glm::dvec3 s_naturalRightElbowPos;
    static const glm::dvec3 s_naturalLeftShoulderPos;
    static const glm::dvec3 s_naturalLeftElbowPos;
    static const glm::dvec3 s_naturalRightHipPos;
    static const glm::dvec3 s_naturalRightKneePos;
    static const glm::dvec3 s_naturalRightFootPos;
    static const glm::dvec3 s_naturalLeftHipPos;
    static const glm::dvec3 s_naturalLeftKneePos;
    static const glm::dvec3 s_naturalLeftFootPos;
    static const glm::dvec3 s_naturalRightPistonRodPos;
    static const glm::dvec3 s_naturalRightPistonHousingPos;
    static const glm::dvec3 s_naturalLeftPistonRodPos;
    static const glm::dvec3 s_naturalLeftPistonHousingPos;
    static const glm::dvec3 s_naturalHeadPos;
    
    // Static limb lengths
    static const double s_upperArmLength;
    static const double s_lowerArmLength;
    static const double s_upperLegLength;
    static const double s_lowerLegLength;
    
    // Compute left positions from right (helper)
    static glm::dvec3 mirrorX(const glm::dvec3& pos) { return glm::dvec3(-pos.x, pos.y, pos.z); }

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
    DigibotResources* m_resources;  // Non-owning pointer to shared resources
    int m_visualMeshSSBOIndex;

    // One geometry and instance per body part
    std::vector<std::weak_ptr<Geometry>> m_bodyPartGeometries;
    std::vector<std::weak_ptr<Instance>> m_bodyPartInstances;

    // IK solver state
    glm::dvec3 m_rightElbowPoint;
    glm::dvec3 m_leftElbowPoint;
    glm::dvec3 m_rightKneePoint;
    glm::dvec3 m_leftKneePoint;
};