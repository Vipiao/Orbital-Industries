// GridCell.cpp
#include "GridCell.h"

void GridCell::forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const {
    for (int i = 0; i < 6; ++i) {
        if (neighbors[i]) {
            callback(neighbors[i]->coordinates);
        }
    }
}