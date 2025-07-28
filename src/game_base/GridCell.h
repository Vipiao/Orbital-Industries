// GridCell.h
#pragma once

#include <glm/glm.hpp>
#include <functional>
#include <array>
#include "../utils/StochasticAnalyzer.h"
#include "CellType.h"

// Forward declaration
class Grid;

/**
 * * @brief Base class for all grid cell types
 * Defines the interface that all grid cells must implement
 */
class GridCell : public IStochasticCell {
private:
    double m_structuralWeakness = -1.0; // Running average structural weakness (-1 = no data yet)
    uint32_t m_cost = 1; // For stochastic analysis
public:
    // Basic cell data that all grid cells have
    CellType type;
    glm::ivec3 coordinates;
    Grid* parentGrid;
    
    // Direct neighbor pointers for fast access (Right, Left, Front, Back, Top, Bottom)
    std::array<GridCell*, 6> neighbors{nullptr};
    
    GridCell(const glm::ivec3& coords, Grid* parent, CellType cellType)
        : coordinates(coords), parentGrid(parent), type(cellType) {}
    
    virtual ~GridCell() = default;
    
    // Structural weakness methods
    virtual double getStructuralWeakness() const { return m_structuralWeakness; }
    virtual void setStructuralWeakness(double weakness) { m_structuralWeakness = weakness; }
    virtual bool hasStructuralData() const { return m_structuralWeakness >= 0.0; }
    
    // Stochastic analysis methods
    virtual uint32_t getCost() const { return m_cost; }
    virtual void setCost(uint32_t cost) { m_cost = cost; }
    
    // Common interface methods
    virtual void forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const override;
    
protected:
    // Prevent direct instantiation
    GridCell() = default;
};