// ThrusterSecondaryCell.cpp
#include "ThrusterSecondaryCell.h"

ThrusterSecondaryCell::ThrusterSecondaryCell(const glm::ivec3& coord, ThrusterBlock* owner)
    : GridCell(coord, TYPE)
    , m_owner(owner)
{
}

std::tuple<double, glm::dvec3, glm::dmat3> ThrusterSecondaryCell::getMassProperties() const {
    return {0.0, glm::dvec3{0.0}, glm::dmat3{0.0}};
}
