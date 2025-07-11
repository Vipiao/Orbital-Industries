#pragma once

#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>
#include <chrono>
#include "HashFunctions.h"
#include "Generator.h"
#include "TimeHandler.h"
#include "AStar.h"

// Cell interface for stochastic analysis
class IStochasticCell {
public:
    int cost = 1; // Cost for stochastic analysis

    virtual ~IStochasticCell() = default;
    
    // Get connected neighbors as positions
    virtual std::vector<glm::ivec3> getConnectedNeighbors() const = 0;
    
    // Check if this cell is valid for pathfinding
    virtual bool isValidForPath() const = 0;

    // Default implementations for cost management
    int getCost() const { return cost; }
    void setCost(int newCost) { cost = newCost; }
};

template<typename CellType>
class StochasticAnalyzer {
public:
    StochasticAnalyzer(const std::unordered_map<glm::ivec3, CellType, IVec3Hash>& cells);
    ~StochasticAnalyzer();
    
    // Create generator for full analysis
    Generator<bool> performAnalysis(TimeHandler& timeHandler);
    
    // Update time limit for current analysis
    void setTimeLimit(std::chrono::high_resolution_clock::time_point newTimeLimit);
    
private:
    // Initialize all cell costs to 1 and populate cache
    void initializeCostsAndCache();
    
    // Increment cost of cells along path
    void incrementPathCosts(const std::vector<glm::ivec3>& path);
    
    // Select two random cells with cost 1, removing invalid ones from cache
    std::pair<bool, std::pair<glm::ivec3, glm::ivec3>> selectRandomCellsWithCostOne();
    
    // Get cell at position
    CellType* getCell(const glm::ivec3& pos) const;
    
    // Get deterministic random index
    size_t getRandomIndex(size_t maxValue);
    
private:
    const std::unordered_map<glm::ivec3, CellType, IVec3Hash>& m_cells;
    
    // Current time limit for analysis
    std::chrono::high_resolution_clock::time_point m_timeLimit;
    
    // Cache of cell positions that may have cost 1 (shrinks over time)
    std::vector<glm::ivec3> m_costOneCellsCache;
    
    // Deterministic random counter
    static size_t s_randomCounter;
};

// Template implementations (must be in header)
template<typename CellType>
StochasticAnalyzer<CellType>::StochasticAnalyzer(const std::unordered_map<glm::ivec3, CellType, IVec3Hash>& cells) 
    : m_cells(cells), m_timeLimit(std::chrono::high_resolution_clock::time_point::max())
{
    initializeCostsAndCache();
}

template<typename CellType>
StochasticAnalyzer<CellType>::~StochasticAnalyzer() = default;

template<typename CellType>
void StochasticAnalyzer<CellType>::initializeCostsAndCache() {
    m_costOneCellsCache.clear();
    
    for (auto& [pos, cell] : m_cells) {
        if (cell.isValidForPath()) {
            const_cast<CellType&>(cell).setCost(1);
            m_costOneCellsCache.push_back(pos);
        }
    }
}

template<typename CellType>
void StochasticAnalyzer<CellType>::setTimeLimit(std::chrono::high_resolution_clock::time_point newTimeLimit) {
    m_timeLimit = newTimeLimit;
}

template<typename CellType>
CellType* StochasticAnalyzer<CellType>::getCell(const glm::ivec3& pos) const {
    auto it = m_cells.find(pos);
    return (it != m_cells.end()) ? const_cast<CellType*>(&it->second) : nullptr;
}

template<typename CellType>
size_t StochasticAnalyzer<CellType>::getRandomIndex(size_t maxValue) {
    if (maxValue == 0) return 0;
    const size_t largePrime = 1000000007;
    return ((s_randomCounter++) * largePrime) % maxValue;
}

template<typename CellType>
Generator<bool> StochasticAnalyzer<CellType>::performAnalysis(TimeHandler& timeHandler) {
    while (m_costOneCellsCache.size() >= 2) {
        // Check if time limit reached
        if (timeHandler.now() >= m_timeLimit) {
            co_yield true; // Yield control, will continue later
            continue;
        }
        
        // Select two random cells with cost 1
        auto [success, cellPair] = selectRandomCellsWithCostOne();
        
        // If we couldn't find two cells, analysis is complete
        if (!success) {
            break;
        }
        
        auto [start, end] = cellPair;
        
        // Test connectivity using existing A* implementation
        bool pathExists = false;
        
        auto pathResult = AStar<glm::ivec3, IVec3Hash>::search(
            start,
            [&](const glm::ivec3& node) {
                // Goal test - return true when we reach the end
                if (node == end) {
                    pathExists = true;
                    return true;
                }
                return false;
            },
            [&](const glm::ivec3& node, auto callback) {
                // Neighbor expansion - call callback for each valid neighbor
                auto cell = getCell(node);
                if (!cell || !cell->isValidForPath()) {
                    return;
                }
                
                auto neighbors = cell->getConnectedNeighbors();
                for (const auto& neighbor : neighbors) {
                    auto neighborCell = getCell(neighbor);
                    if (neighborCell && neighborCell->isValidForPath()) {
                        double cost = static_cast<double>(neighborCell->getCost());
                        callback(neighbor, cost);
                    }
                }
            },
            [&](const glm::ivec3& node) {
                // Heuristic function - Manhattan distance
                return static_cast<double>(
                    std::abs(node.x - end.x) + 
                    std::abs(node.y - end.y) + 
                    std::abs(node.z - end.z)
                );
            }
        );
        
        if (pathExists) {
            // Use the actual path found by A*
            std::vector<glm::ivec3> path = pathResult.path;
            incrementPathCosts(path);
        } else {
            incrementPathCosts({start, end});
        }
    }
}

template<typename CellType>
void StochasticAnalyzer<CellType>::incrementPathCosts(const std::vector<glm::ivec3>& path) {
    for (const auto& pos : path) {
        auto cell = getCell(pos);
        if (cell) {
            cell->setCost(cell->getCost() + 1);
        }
    }
}

template<typename CellType>
std::pair<bool, std::pair<glm::ivec3, glm::ivec3>> StochasticAnalyzer<CellType>::selectRandomCellsWithCostOne() {
    // Check if we have enough cells
    if (m_costOneCellsCache.size() < 2) {
        return {false, {}};
    }
    
    glm::ivec3 firstCell;
    glm::ivec3 secondCell;
    bool foundFirst = false;
    bool foundSecond = false;
    
    // Select first valid cell
    while (!foundFirst && !m_costOneCellsCache.empty()) {
        size_t firstIndex = getRandomIndex(m_costOneCellsCache.size());
        auto cell = getCell(m_costOneCellsCache[firstIndex]);
        
        if (cell && cell->isValidForPath() && cell->getCost() == 1) {
            firstCell = m_costOneCellsCache[firstIndex];
            foundFirst = true;
        } else {
            // Remove invalid cell by swapping with last and popping
            m_costOneCellsCache[firstIndex] = m_costOneCellsCache.back();
            m_costOneCellsCache.pop_back();
        }
    }
    
    // Check if we still have enough cells for a second selection
    if (!foundFirst || m_costOneCellsCache.size() < 2) {
        return {false, {}};
    }
    
    // Select second valid cell (different from first)
    while (!foundSecond && m_costOneCellsCache.size() >= 2) {
        size_t secondIndex = getRandomIndex(m_costOneCellsCache.size());
        auto cell = getCell(m_costOneCellsCache[secondIndex]);
        
        if (cell && cell->isValidForPath() && cell->getCost() == 1 && m_costOneCellsCache[secondIndex] != firstCell) {
            secondCell = m_costOneCellsCache[secondIndex];
            foundSecond = true;
        } else {
            // Remove invalid cell by swapping with last and popping
            m_costOneCellsCache[secondIndex] = m_costOneCellsCache.back();
            m_costOneCellsCache.pop_back();
        }
    }
    
    if (foundFirst && foundSecond) {
        return {true, {firstCell, secondCell}};
    } else {
        return {false, {}};
    }
}