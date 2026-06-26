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

std::vector<glm::dvec3> GridCell::getLocalVertices() const {
    return {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {1.0, 1.0, 1.0}
    };
}

size_t GridCell::computeHash() const {
    size_t hash = 0;
    
    hash = Hash::combineHashes(hash, std::hash<int>{}(static_cast<int>(type)));
    hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(coordinates));
    hash = Hash::combineHashes(hash, std::hash<int>{}(cost));
    hash = Hash::combineHashes(hash, std::hash<double>{}(structuralWeakness));
    hash = Hash::combineHashes(hash, std::hash<uint32_t>{}(uniqueId));
    
    return hash;
}