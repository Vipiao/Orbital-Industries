// DigibotGraphics.cpp
#include "DigibotGraphics.h"
#include "DigibotResources.h"
#include "../ArticulationUtils.h"
#include "../../graphics/GraphicsEngine.h"
#include "../../graphics/instanceHandler/InstanceHandler.h"
#include "../../graphics/SSBOManager.h"
#include <iostream>

// Static skeleton measurements (T-pose natural positions)
const glm::dvec3 DigibotGraphics::s_naturalRightShoulderPos = glm::dvec3(0.26788, -0.044638, 1.47241);
const glm::dvec3 DigibotGraphics::s_naturalRightElbowPos = glm::dvec3(0.687425, -0.044638, 1.47241);
const glm::dvec3 DigibotGraphics::s_naturalLeftShoulderPos = glm::dvec3(-0.26788, -0.044638, 1.47241);
const glm::dvec3 DigibotGraphics::s_naturalLeftElbowPos = glm::dvec3(-0.687425, -0.044638, 1.47241);

const glm::dvec3 DigibotGraphics::s_naturalRightHipPos = glm::dvec3(0.177658, 0.061087, 1.02951);
const glm::dvec3 DigibotGraphics::s_naturalRightKneePos = glm::dvec3(0.180854, -0.274169, 0.530615);
const glm::dvec3 DigibotGraphics::s_naturalRightFootPos = glm::dvec3(0.179225, 0.051327, 0.059608);
const glm::dvec3 DigibotGraphics::s_naturalLeftHipPos = glm::dvec3(-0.177658, 0.061087, 1.02951);
const glm::dvec3 DigibotGraphics::s_naturalLeftKneePos = glm::dvec3(-0.180854, -0.274169, 0.530615);
const glm::dvec3 DigibotGraphics::s_naturalLeftFootPos = glm::dvec3(-0.179225, 0.051327, 0.059608);
const glm::dvec3 DigibotGraphics::s_naturalRightPistonRodPos = glm::dvec3(0.180853, -0.130187, 0.67836);
const glm::dvec3 DigibotGraphics::s_naturalRightPistonHousingPos = glm::dvec3(0.180853, -0.130187, 0.324156);
const glm::dvec3 DigibotGraphics::s_naturalLeftPistonRodPos = glm::dvec3(-0.180853, -0.130187, 0.67836);
const glm::dvec3 DigibotGraphics::s_naturalLeftPistonHousingPos = glm::dvec3(-0.180853, -0.130187, 0.324156);
const glm::dvec3 DigibotGraphics::s_naturalHeadPos = glm::dvec3(0.0, 0.0, 1.39974);

// Static limb lengths
const double DigibotGraphics::s_upperArmLength = glm::length(
    DigibotGraphics::s_naturalRightElbowPos - DigibotGraphics::s_naturalRightShoulderPos);
const double DigibotGraphics::s_lowerArmLength = glm::length(
    glm::dvec3(1.09582, -0.05088, 1.47241) - DigibotGraphics::s_naturalRightElbowPos);
const double DigibotGraphics::s_upperLegLength = glm::length(
    DigibotGraphics::s_naturalRightKneePos - DigibotGraphics::s_naturalRightHipPos);
const double DigibotGraphics::s_lowerLegLength = glm::length(
    DigibotGraphics::s_naturalRightFootPos - DigibotGraphics::s_naturalRightKneePos);

DigibotGraphics::DigibotGraphics(GraphicsEngine* graphics, DigibotResources* resources)
    : m_graphics(graphics)
    , m_resources(resources)
    , m_visualMeshSSBOIndex(-1)
    , m_leftElbowPoint(0.0, 0.0, 0.0)
    , m_rightElbowPoint(0.0, 0.0, 0.0)
    , m_leftKneePoint(0.0, 0.0, 0.0)
    , m_rightKneePoint(0.0, 0.0, 0.0)
{
    if (!m_resources) {
        throw std::runtime_error("DigibotResources cannot be null");
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
            throw std::runtime_error("DigibotGraphics: Shared geometry is invalid for body part " + std::to_string(i));
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

DigibotGraphics::~DigibotGraphics() {
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

void DigibotGraphics::initializeInstanceTransforms() {
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

void DigibotGraphics::updateWorldTransform(
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

void DigibotGraphics::updateBodyPartPositions(const DigibotTargetPose& targetPose) {
    // ========== SETUP ARRAYS ==========
    glm::dvec3* elbows[2] = {&m_rightElbowPoint, &m_leftElbowPoint};
    glm::dvec3* knees[2] = {&m_rightKneePoint, &m_leftKneePoint};
    
    BodyPart upperArms[2] = {RIGHT_UPPER_ARM, LEFT_UPPER_ARM};
    BodyPart lowerArms[2] = {RIGHT_LOWER_ARM, LEFT_LOWER_ARM};
    BodyPart upperLegs[2] = {RIGHT_UPPER_LEG, LEFT_UPPER_LEG};
    BodyPart lowerLegs[2] = {RIGHT_LOWER_LEG, LEFT_LOWER_LEG};
    BodyPart feet[2] = {RIGHT_FOOT, LEFT_FOOT};
    BodyPart pistonRods[2] = {RIGHT_PISTON_ROD, LEFT_PISTON_ROD};
    BodyPart pistonHousings[2] = {RIGHT_PISTON_HOUSING, LEFT_PISTON_HOUSING};
    
    glm::dvec3 shoulders[2] = {s_naturalRightShoulderPos, s_naturalLeftShoulderPos};
    glm::dvec3 naturalElbows[2] = {s_naturalRightElbowPos, s_naturalLeftElbowPos};
    glm::dvec3 hips[2] = {s_naturalRightHipPos, s_naturalLeftHipPos};
    glm::dvec3 naturalKnees[2] = {s_naturalRightKneePos, s_naturalLeftKneePos};
    glm::dvec3 naturalFeet[2] = {s_naturalRightFootPos, s_naturalLeftFootPos};
    glm::dvec3 naturalPistonRods[2] = {s_naturalRightPistonRodPos, s_naturalLeftPistonRodPos};
    glm::dvec3 naturalPistonHousings[2] = {s_naturalRightPistonHousingPos, s_naturalLeftPistonHousingPos};
    
    glm::dvec3 handTargets[2] = {targetPose.rightHand.position, targetPose.leftHand.position};
    glm::dvec3 footTargets[2] = {targetPose.rightFoot.position, targetPose.leftFoot.position};
    
    glm::dvec3 elbowPreferredDirections[2] = {
        glm::normalize(glm::dvec3(1.0, 0.0, -1.0)),   // Right: +X right, -Z down
        glm::normalize(glm::dvec3(-1.0, 0.0, -1.0))   // Left: -X left, -Z down
    };
    
    glm::dvec3 kneePreferredDirections[2] = {
        glm::dvec3(0.0, -1.0, 0.0),   // Right: backward (knees bend backwards)
        glm::dvec3(0.0, -1.0, 0.0)    // Left: backward (knees bend backwards)
    };
    
    // ========== IK CONSTRAINT SOLVING ==========
    int numIKIterations = 1;
    for (int iteration = 0; iteration < numIKIterations; ++iteration) {
        // Arm IK
        for (int ii = 0; ii < 2; ++ii) {
            // Constraint 1: Elbow wants to be in the lower right/left direction
            *elbows[ii] = ArticulationUtils::applyDirectionNudge(*elbows[ii], elbowPreferredDirections[ii], 0.02);

            // Constraint 2: Elbow must be upperArmLength from shoulder
            *elbows[ii] = ArticulationUtils::applyDistanceConstraint(*elbows[ii], shoulders[ii], s_upperArmLength);

            // Constraint 3: Elbow must be lowerArmLength from hand target
            *elbows[ii] = ArticulationUtils::applyDistanceConstraint(*elbows[ii], handTargets[ii], s_lowerArmLength);
        }
        
        // Leg IK
        for (int ii = 0; ii < 2; ++ii) {
            // Constraint 1: Knee wants to be in the backward direction
            *knees[ii] = ArticulationUtils::applyDirectionNudge(*knees[ii], kneePreferredDirections[ii], 0.02);

            // Constraint 2: Knee must be upperLegLength from hip
            *knees[ii] = ArticulationUtils::applyDistanceConstraint(*knees[ii], hips[ii], s_upperLegLength);

            // Constraint 3: Knee must be lowerLegLength from foot target
            *knees[ii] = ArticulationUtils::applyDistanceConstraint(*knees[ii], footTargets[ii], s_lowerLegLength);
        }
    }
    
    // ========== ARTICULATION ==========
    
    // Arm articulation
    glm::dquat rightArmCorrectionQuat = glm::angleAxis(glm::radians(90.0), glm::dvec3(0.0, 0.0, 1.0));
    glm::dquat leftArmCorrectionQuat = glm::conjugate(rightArmCorrectionQuat);
    glm::dquat armCorrectionQuats[2] = {rightArmCorrectionQuat, leftArmCorrectionQuat};
    
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
            shoulders[ii],
            *elbows[ii] - shoulders[ii],
            up,
            armCorrectionQuats[ii],
            shoulders[ii]
        );

        // Update lower arm
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[lowerArms[ii]],
            m_bodyPartInstances[lowerArms[ii]],
            *elbows[ii],
            handTargets[ii] - *elbows[ii],
            up,
            armCorrectionQuats[ii],
            naturalElbows[ii]
        );
    }

    // Leg articulation
    for (int ii = 0; ii < 2; ++ii) {
        // Up vector for legs is simply positive Y
        glm::dvec3 up = glm::dvec3(0.0, 1.0, 0.0);

        // Calculate correction quaternions based on natural pose orientations
        glm::dvec3 upperLegNaturalForward = glm::normalize(naturalKnees[ii] - hips[ii]);
        glm::dquat upperLegCorrection = ArticulationUtils::quatLookAtYForward(upperLegNaturalForward, up);

        glm::dvec3 lowerLegNaturalForward = glm::normalize(naturalFeet[ii] - naturalKnees[ii]);
        glm::dquat lowerLegCorrection = ArticulationUtils::quatLookAtYForward(lowerLegNaturalForward, up);
 
        glm::dquat footCorrection = glm::angleAxis(glm::radians(90.0), glm::dvec3(1.0, 0.0, 0.0));

        // Update upper leg (hip to knee)
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[upperLegs[ii]],
            m_bodyPartInstances[upperLegs[ii]],
            hips[ii],
            *knees[ii] - hips[ii],
            up,
            upperLegCorrection,
            hips[ii]
        );

        // Update lower leg (knee to foot)
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[lowerLegs[ii]],
            m_bodyPartInstances[lowerLegs[ii]],
            *knees[ii],
            footTargets[ii] - *knees[ii],
            up,
            lowerLegCorrection,
            naturalKnees[ii]
        );

        // Update foot (at foot position, pointing down)
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[feet[ii]],
            m_bodyPartInstances[feet[ii]],
            footTargets[ii],
            glm::dvec3(0.0, 0.0, -1.0),
            up,
            footCorrection,
            naturalFeet[ii]
        );
    }

    // Piston articulation
    for (int ii = 0; ii < 2; ++ii) {
        // Get upper and lower leg instances to read their transforms
        auto upperLegInstance = m_bodyPartInstances[upperLegs[ii]].lock();
        auto lowerLegInstance = m_bodyPartInstances[lowerLegs[ii]].lock();

        if (!upperLegInstance || !lowerLegInstance) {
            continue;
        }

        // Transform natural rod position by upper leg's current transform
        glm::dvec3 transformedRodPos = upperLegInstance->m_localOrientation * naturalPistonRods[ii] 
                                     + upperLegInstance->m_localPosition;

        // Transform natural housing position by lower leg's current transform
        glm::dvec3 transformedHousingPos = lowerLegInstance->m_localOrientation * naturalPistonHousings[ii]
                                         + lowerLegInstance->m_localPosition;

        // Up vector for pistons
        glm::dvec3 up = glm::dvec3(0.0, 1.0, 0.0);

        // Calculate correction quaternions based on natural pose orientations
        glm::dvec3 rodNaturalForward = glm::normalize(naturalPistonHousings[ii] - naturalPistonRods[ii]);
        glm::dquat rodCorrection = ArticulationUtils::quatLookAtYForward(rodNaturalForward, up);

        glm::dvec3 housingNaturalForward = glm::normalize(naturalPistonRods[ii] - naturalPistonHousings[ii]);
        glm::dquat housingCorrection = ArticulationUtils::quatLookAtYForward(housingNaturalForward, up);

        // Update piston rod (attached to upper leg, points toward housing)
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[pistonRods[ii]],
            m_bodyPartInstances[pistonRods[ii]],
            transformedRodPos,
            transformedHousingPos - transformedRodPos,
            up,
            rodCorrection,
            naturalPistonRods[ii]
        );

        // Update piston housing (attached to lower leg, points toward rod)
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[pistonHousings[ii]],
            m_bodyPartInstances[pistonHousings[ii]],
            transformedHousingPos,
            transformedRodPos - transformedHousingPos,
            up,
            housingCorrection,
            naturalPistonHousings[ii]
        );
    }

    // Head articulation
    {
        // Calculate direction and up from head orientation
        glm::dvec3 headDirection = targetPose.headOrientation * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 headUp = targetPose.headOrientation * glm::dvec3(0.0, 0.0, 1.0);

        // Identity correction quaternion
        glm::dquat headCorrection = glm::dquat(1.0, 0.0, 0.0, 0.0);

        // Update head
        ArticulationUtils::updateArticulatedBodyPart(
            m_bodyPartGeometries[HEAD],
            m_bodyPartInstances[HEAD],
            s_naturalHeadPos,
            headDirection,
            headUp,
            headCorrection,
            s_naturalHeadPos
        );
    }
}