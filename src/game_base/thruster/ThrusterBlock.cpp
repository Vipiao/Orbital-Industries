// ThrusterBlock.cpp
#include "ThrusterBlock.h"
#include <glm/glm.hpp>

ThrusterBlock::ThrusterBlock(const glm::ivec3& anchorCoord)
    : GridCell(anchorCoord, TYPE)
    , m_secondCoord(anchorCoord + glm::ivec3{0, 1, 0})
{
}

std::tuple<double, glm::dvec3, glm::dmat3> ThrusterBlock::getMassProperties() const {
    constexpr double density = 1.5e3;
    constexpr double volume  = 2.0;   // 1x2x1 grid units
    constexpr double mass    = density * volume; // 3000 kg

    // Center of the 1x2x1 box, relative to anchor coord
    const glm::dvec3 com{0.5, 1.0, 0.5};

    // Inertia of a box (dims dx=1, dy=2, dz=1) about its center: I_axis = M/12*(other²+other²)
    constexpr double ixx = mass / 12.0 * (4.0 + 1.0); // dy²+dz² = 4+1
    constexpr double iyy = mass / 12.0 * (1.0 + 1.0); // dx²+dz² = 1+1
    constexpr double izz = mass / 12.0 * (1.0 + 4.0); // dx²+dy² = 1+4

    glm::dmat3 inertia{0.0};
    inertia[0][0] = ixx;
    inertia[1][1] = iyy;
    inertia[2][2] = izz;

    return {mass, com, inertia};
}
