// CockpitBlock.cpp
#include "CockpitBlock.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

CockpitBlock::CockpitBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation)
    : BlockAnchor{anchorCoord, TYPE, orientation}
{
}

std::vector<BlockGeometryPart> CockpitBlock::geometryParts() {
    return {
        {"../media/models/cockpit/frame.obj",      1.0},
        {"../media/models/cockpit/door_left.obj",  1.0},
        {"../media/models/cockpit/door_right.obj", 1.0},
        {"../media/models/cockpit/glass.obj",      0.3},
    };
}

std::vector<glm::ivec3> CockpitBlock::footprintOffsets(const glm::dquat& /*orientation*/) {
    return {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
        {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}
    };
}

std::vector<glm::ivec3> CockpitBlock::secondaryCoords() const {
    return {
        coordinates + glm::ivec3{1, 0, 0},
        coordinates + glm::ivec3{0, 1, 0},
        coordinates + glm::ivec3{0, 0, 1},
        coordinates + glm::ivec3{1, 1, 0},
        coordinates + glm::ivec3{1, 0, 1},
        coordinates + glm::ivec3{0, 1, 1},
        coordinates + glm::ivec3{1, 1, 1},
    };
}

std::tuple<double, glm::dvec3, glm::dmat3> CockpitBlock::getMassProperties() const {
    constexpr double density = 1.5e3;
    constexpr double volume  = 8.0;          // 2x2x2 grid units
    constexpr double mass    = density * volume; // 12000 kg

    // Centre of the 2x2x2 box relative to the anchor coord
    const glm::dvec3 com{1.0, 1.0, 1.0};

    // Inertia tensor for a uniform solid box with side length 2:
    // I = M/12 * (a² + b²) where a,b are the two perpendicular side lengths
    constexpr double I = mass / 12.0 * (4.0 + 4.0); // 8000 kg·m²

    glm::dmat3 canonical{0.0};
    canonical[0][0] = I;
    canonical[1][1] = I;
    canonical[2][2] = I;

    // Rotate into grid-local space (cube stays isotropic, but done for consistency)
    glm::dmat3 R{glm::mat3_cast(m_orientation)};
    glm::dmat3 inertia{R * canonical * glm::transpose(R)};

    return {mass, com, inertia};
}
