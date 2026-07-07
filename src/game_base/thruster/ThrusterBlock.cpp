// ThrusterBlock.cpp
#include "ThrusterBlock.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

std::vector<BlockGeometryPart> ThrusterBlock::geometryParts() {
    return {{"../media/models/thruster/thruster_v2.obj", 1.0}};
}

std::vector<glm::ivec3> ThrusterBlock::footprintOffsets(const glm::dquat& orientation) {
    return {{0, 0, 0}, dominantAxis(orientation)};
}

glm::ivec3 ThrusterBlock::dominantAxis(const glm::dquat& orientation) {
    glm::dvec3 rotated = orientation * glm::dvec3{0.0, 1.0, 0.0};
    glm::dvec3 abs = glm::abs(rotated);
    if (abs.x >= abs.y && abs.x >= abs.z)
        return {rotated.x > 0.0 ? 1 : -1, 0, 0};
    if (abs.y >= abs.z)
        return {0, rotated.y > 0.0 ? 1 : -1, 0};
    return {0, 0, rotated.z > 0.0 ? 1 : -1};
}

glm::ivec3 ThrusterBlock::secondCoord(const glm::ivec3& anchorCoord, const glm::dquat& orientation) {
    return anchorCoord + dominantAxis(orientation);
}

ThrusterBlock::ThrusterBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation)
    : BlockAnchor{anchorCoord, TYPE, orientation}
{
}

std::vector<glm::ivec3> ThrusterBlock::secondaryCoords() const {
    return {secondCoord(coordinates, m_orientation)};
}

std::tuple<double, glm::dvec3, glm::dmat3> ThrusterBlock::getMassProperties() const {
    constexpr double density = 1.5e3;
    constexpr double volume  = 2.0;   // 1x2x1 grid units
    constexpr double mass    = density * volume; // 3000 kg

    // Center of the 1x2x1 box in cell-local space: midpoint between the two unit cubes.
    // Anchor cube center: {0.5, 0.5, 0.5}; secondary cube center: axis + {0.5, 0.5, 0.5}.
    glm::dvec3 axis{dominantAxis(m_orientation)};
    const glm::dvec3 com{glm::dvec3{0.5, 0.5, 0.5} + axis * 0.5};

    // Canonical inertia tensor for a 1x2x1 box aligned with +Y (model space).
    // I_axis = M/12 * (sum of other two squared dimensions)
    constexpr double ixx = mass / 12.0 * (4.0 + 1.0); // dy²+dz² = 4+1
    constexpr double iyy = mass / 12.0 * (1.0 + 1.0); // dx²+dz²
    constexpr double izz = mass / 12.0 * (1.0 + 4.0); // dx²+dy²

    glm::dmat3 canonical{0.0};
    canonical[0][0] = ixx;
    canonical[1][1] = iyy;
    canonical[2][2] = izz;

    // Rotate canonical tensor into grid-local space.
    glm::dmat3 R{glm::mat3_cast(m_orientation)};
    glm::dmat3 inertia{R * canonical * glm::transpose(R)};

    return {mass, com, inertia};
}
