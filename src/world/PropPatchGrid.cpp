// PropPatchGrid.cpp
#include "PropPatchGrid.h"
#include "CubeFaces.h"
#include <algorithm>
#include <cassert>
#include <cmath>

namespace {

constexpr double k_quarterTurn{1.5707963267948966};

// The most the grid angle turns per angle moved over the sphere: 1 / sqrt(2/3)
// at a face's corners, rounded up
constexpr double k_angleStretch{1.25};

}  // namespace

PropPatchGrid::PropPatchGrid(double radiusMetres, double patchMetres)
    : m_radius{radiusMetres},
      m_patchesPerEdge{
          static_cast<uint32_t>(std::lround(k_quarterTurn * radiusMetres / patchMetres))} {
    assert(m_patchesPerEdge > 0 && "a prop patch must be smaller than a face");
}

glm::dvec3 PropPatchGrid::gridDirection(uint32_t face, double column, double row) const {
    assert(face < CubeFaces::k_faceCount && "face out of range");
    assert(column >= 0.0 && column <= m_patchesPerEdge && row >= 0.0 &&
           row <= m_patchesPerEdge && "off the face");
    // Equal steps of angle, then where that meets the face, 0 .. 1 across it
    const double angleU{k_quarterTurn * (column / m_patchesPerEdge - 0.5)};
    const double angleV{k_quarterTurn * (row / m_patchesPerEdge - 0.5)};
    const double acrossU{0.5 * (std::tan(angleU) + 1.0)};
    const double acrossV{0.5 * (std::tan(angleV) + 1.0)};

    const int faceIndex{static_cast<int>(face)};
    return glm::normalize(CubeFaces::faceOrigin(faceIndex) +
                          acrossU * CubeFaces::faceUAxis(faceIndex) +
                          acrossV * CubeFaces::faceVAxis(faceIndex));
}

glm::dvec3 PropPatchGrid::patchOrigin(const PropPatchKey& key) const {
    return m_radius * gridDirection(key.m_face, key.m_column, key.m_row);
}

std::vector<PropPatchKey> PropPatchGrid::patchesNear(const glm::dvec3& direction,
                                                     double reachMetres) const {
    std::vector<PropPatchKey> near{};

    assert(std::abs(glm::length(direction) - 1.0) < 1e-9 && "direction must be unit length");
    // Patches either side worth measuring, plus one for the patch's own width
    const double patchesPerRadian{m_patchesPerEdge / k_quarterTurn};
    const int64_t window{
        static_cast<int64_t>(
            std::ceil(k_angleStretch * reachMetres / m_radius * patchesPerRadian)) + 1};
    const int64_t lastPatch{static_cast<int64_t>(m_patchesPerEdge) - 1};

    for (int face{0}; face < CubeFaces::k_faceCount; ++face) {
        const glm::dvec3& origin{CubeFaces::faceOrigin(face)};
        const glm::dvec3& uAxis{CubeFaces::faceUAxis(face)};
        const glm::dvec3& vAxis{CubeFaces::faceVAxis(face)};

        // A face turned away is a quarter turn or more off
        const glm::dvec3 outward{glm::normalize(glm::cross(uAxis, vAxis))};
        const double facing{glm::dot(direction, outward)};
        if (facing <= 0.0) {
            continue;
        }

        // May fall past the face's edge; the window is clamped to the face
        const glm::dvec3 fromCorner{direction / facing - origin};
        const double acrossU{glm::dot(fromCorner, uAxis) / glm::dot(uAxis, uAxis)};
        const double acrossV{glm::dot(fromCorner, vAxis) / glm::dot(vAxis, vAxis)};
        const double column{
            (std::atan(2.0 * acrossU - 1.0) / k_quarterTurn + 0.5) * m_patchesPerEdge};
        const double row{
            (std::atan(2.0 * acrossV - 1.0) / k_quarterTurn + 0.5) * m_patchesPerEdge};

        const int64_t middleColumn{static_cast<int64_t>(std::floor(column))};
        const int64_t middleRow{static_cast<int64_t>(std::floor(row))};
        const int64_t firstColumn{std::max<int64_t>(0, middleColumn - window)};
        const int64_t lastColumn{std::min(lastPatch, middleColumn + window)};
        const int64_t firstRow{std::max<int64_t>(0, middleRow - window)};
        const int64_t lastRow{std::min(lastPatch, middleRow + window)};

        for (int64_t patchColumn{firstColumn}; patchColumn <= lastColumn; ++patchColumn) {
            for (int64_t patchRow{firstRow}; patchRow <= lastRow; ++patchRow) {
                const PropPatchKey key{static_cast<uint32_t>(face),
                                       static_cast<uint32_t>(patchColumn),
                                       static_cast<uint32_t>(patchRow)};
                if (nearestDistance(key, direction) <= reachMetres) {
                    near.push_back(key);
                }
            }
        }
    }

    return near;
}

double PropPatchGrid::nearestDistance(const PropPatchKey& key,
                                      const glm::dvec3& direction) const {
    const glm::dvec3 middle{gridDirection(key.m_face, key.m_column + 0.5, key.m_row + 0.5)};

    // Chords for arcs: over kilometres they differ by a part in 10^8
    return m_radius * glm::length(direction - middle) - halfWidth(key);
}

uint32_t PropPatchGrid::maxPatchesWithin(double reachMetres) const {
    // The extremes lie at a face's middle, edge middle and corner
    const uint32_t middle{m_patchesPerEdge / 2};
    const PropPatchKey extremes[3]{{0, middle, middle}, {0, middle, 0}, {0, 0, 0}};

    double smallestArea{area(extremes[0])};
    double widest{0.0};
    for (const PropPatchKey& key : extremes) {
        smallestArea = std::min(smallestArea, area(key));
        widest = std::max(widest, halfWidth(key));
    }

    const double pi{2.0 * k_quarterTurn};
    const double circle{reachMetres + 2.0 * widest};
    return static_cast<uint32_t>(std::ceil(pi * circle * circle / smallestArea));
}

double PropPatchGrid::halfWidth(const PropPatchKey& key) const {
    const glm::dvec3 middle{gridDirection(key.m_face, key.m_column + 0.5, key.m_row + 0.5)};

    double furthest{0.0};
    for (uint32_t corner{0}; corner < 4; ++corner) {
        const glm::dvec3 cornerDirection{
            gridDirection(key.m_face, key.m_column + (corner & 1u), key.m_row + (corner >> 1))};
        furthest = std::max(furthest, glm::length(cornerDirection - middle));
    }
    return m_radius * furthest;
}

double PropPatchGrid::area(const PropPatchKey& key) const {
    const glm::dvec3 lowLow{patchOrigin(key)};
    const glm::dvec3 highLow{patchOrigin({key.m_face, key.m_column + 1, key.m_row})};
    const glm::dvec3 lowHigh{patchOrigin({key.m_face, key.m_column, key.m_row + 1})};
    const glm::dvec3 highHigh{patchOrigin({key.m_face, key.m_column + 1, key.m_row + 1})};

    return 0.5 * glm::length(glm::cross(highLow - lowLow, lowHigh - lowLow)) +
           0.5 * glm::length(glm::cross(highLow - highHigh, lowHigh - highHigh));
}
