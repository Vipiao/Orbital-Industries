// HashFunctions.h
#pragma once

#include <glm/glm.hpp>
#include <cstddef>

// Hash function for glm::ivec3 with bit shifting to prevent coordinate permutation collisions
struct IVec3Hash {
    size_t operator()(const glm::ivec3& coord) const {
        static_assert(sizeof(size_t) == 8, "size_t must be 64-bit");
        size_t hash = static_cast<size_t>(coord.x) ^ 
                      (static_cast<size_t>(coord.y) << 16) ^ 
                      (static_cast<size_t>(coord.z) << 32);
        hash = hash * 73856093;
        return hash;
    }
};