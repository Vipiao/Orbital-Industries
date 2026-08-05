// SecondaryCell.cpp
#include "SecondaryCell.h"

SecondaryCell::SecondaryCell(const glm::ivec3& coord, const glm::ivec3& anchorCoord)
    : GridCell{coord, CellType::SECONDARY}, m_anchorCoord{anchorCoord}
{
}

std::tuple<double, glm::dvec3, glm::dmat3> SecondaryCell::getMassProperties() const {
    return {0.0, glm::dvec3{0.0}, glm::dmat3{0.0}};
}
