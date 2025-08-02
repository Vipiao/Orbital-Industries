// GridCell.h
#pragma once

#include <glm/glm.hpp>
#include <functional>
#include <array>
#include "../utils/IHashable.h"
#include "../utils/StochasticAnalyzer.h"
#include "CellType.h"

/**
 * * @brief Base class for all grid cell types
 * Defines the interface that all grid cells must implement
 */
class GridCell : public IStochasticCell, public IHashable {
public:
    // Basic cell data that all grid cells have
    CellType type;
    uint64_t uniqueId;
    glm::ivec3 coordinates;
    
    // Direct neighbor pointers for fast access (Right, Left, Front, Back, Top, Bottom)
    std::array<GridCell*, 6> neighbors{nullptr};

    // Structural analysis data
    double structuralWeakness = -1.0; // Running average structural weakness (-1 = no data yet)
    
    GridCell(const glm::ivec3& coords, CellType cellType)
        : coordinates(coords), type(cellType), uniqueId(getNextId()) {}
    
    virtual ~GridCell() = default;
    
    // Common interface methods
    virtual void forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const override;
    
    // IHashable interface
    virtual size_t computeHash() const override;

protected:
    // Prevent direct instantiation
    GridCell() = default;

private:
    static uint64_t nextId;
    static uint64_t getNextId() { return nextId++; }
};