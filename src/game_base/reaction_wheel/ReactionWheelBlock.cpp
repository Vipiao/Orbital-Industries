// ReactionWheelBlock.cpp
#include "ReactionWheelBlock.h"
#include "utils/HashFunctions.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

std::vector<BlockGeometryPart> ReactionWheelBlock::geometryParts() {
    return {
        {"../media/models/reaction_wheel/frame.obj", 1.0},
        {"../media/models/reaction_wheel/glass.obj", 0.3},
    };
}

std::vector<glm::ivec3> ReactionWheelBlock::footprintOffsets(const glm::dquat& /*orientation*/) {
    return {{0, 0, 0}};
}

ReactionWheelBlock::ReactionWheelBlock(const glm::ivec3& anchorCoord,
                                       const glm::dquat& orientation)
    : BlockAnchor{anchorCoord, TYPE, orientation}
{
}

std::vector<glm::ivec3> ReactionWheelBlock::secondaryCoords() const {
    return secondariesOf(footprintOffsets(m_orientation));
}

std::tuple<double, glm::dvec3, glm::dmat3> ReactionWheelBlock::getMassProperties() const {
    constexpr double density = 1.5e3;
    constexpr double volume  = 1.0;              // 1x1x1 grid units
    constexpr double mass    = density * volume; // 1500 kg

    // Inertia of a uniform unit cube: isotropic, so orientation does not enter.
    constexpr double I = mass / 12.0 * (1.0 + 1.0);

    glm::dmat3 inertia{0.0};
    inertia[0][0] = I;
    inertia[1][1] = I;
    inertia[2][2] = I;

    return {mass, MODEL_CENTRE, inertia};
}

size_t ReactionWheelBlock::computeHash() const {
    // The command drives torques, so it is simulation state and must feed the hash.
    return Hash::combineHashes(GridCell::computeHash(),
                               Hash::DVec3Hash{}(m_torqueCommand));
}
