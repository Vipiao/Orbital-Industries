// GridCell.h
#pragma once

#include <glm/glm.hpp>
#include <functional>
#include <array>
#include <vector>
#include "../utils/IHashable.h"
#include "../utils/StochasticAnalyzer.h"
#include "CellType.h"
#include <tuple>

/**
 * * @brief Base class for all grid cell types
 * Defines the interface that all grid cells must implement
 */
class GridCell : public IStochasticCell, public IHashable {
public:
    // Basic cell data that all grid cells have
    glm::ivec3 coordinates;
    CellType type;
    uint64_t uniqueId;
    
    // Direct neighbor pointers for fast access (Right, Left, Front, Back, Top, Bottom)
    std::array<GridCell*, 6> neighbors{nullptr};

    // Structural analysis data
    double structuralWeakness = -1.0; // Running average structural weakness (-1 = no data yet)
    
    // Color in RGBA format (used for HSV transform in shader)
    glm::dvec4 m_color{1.0, 1.0, 1.0, 1.0}; // Default white

    GridCell(const glm::ivec3& coords, CellType cellType)
        : coordinates(coords), type(cellType), uniqueId(getNextId()) {}
    
    virtual ~GridCell() = default;
    
    // Common interface methods
    virtual void forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const override;
    
    /**
     * @brief Get mass properties of this cell
     * @return Tuple of (mass, local center of mass, local inertia tensor)
     */
    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const = 0;

    /**
     * @brief Get local vertices of this cell (in [0,1] space relative to cell coord)
     * Default returns a unit cube. Override for non-cube shapes.
     */
    virtual std::vector<glm::dvec3> getLocalVertices() const;

    // IHashable interface
    virtual size_t computeHash() const override;

protected:
    // Prevent direct instantiation
    GridCell() = default;

private:
    static uint64_t nextId;
    static uint64_t getNextId() { return nextId++; }
};