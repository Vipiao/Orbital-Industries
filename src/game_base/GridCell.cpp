// GridCell.cpp
#include "GridCell.h"
#include "../utils/HashFunctions.h"

// Initialize static member
uint64_t GridCell::nextId = 0;

void GridCell::forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const {
    for (int i = 0; i < 6; ++i) {
        if (neighbors[i]) {
            callback(neighbors[i]->coordinates);
        }
    }
}

size_t GridCell::computeHash() const {
    size_t hash = 0;
    
    hash = combineHashes(hash, std::hash<int>{}(static_cast<int>(type)));
    hash = combineHashes(hash, IVec3Hash{}(coordinates));
    hash = combineHashes(hash, std::hash<int>{}(cost));
    hash = combineHashes(hash, std::hash<double>{}(structuralWeakness));
    hash = combineHashes(hash, std::hash<uint32_t>{}(uniqueId));
    
    return hash;
}