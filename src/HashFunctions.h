// HashFunctions.h
#pragma once

#include <glm/glm.hpp>
#include <cstddef>
#include <utility>
#include <functional>

// Compile-time string hash using FNV-1a algorithm
constexpr int hashColliderName(const char* name) {
    unsigned int hash = 2166136261u;
    while (*name) {
        hash ^= *name++;
        hash *= 16777619;
    }
    return static_cast<int>(hash);
}

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

// Hash function for std::pair<uintptr_t, uintptr_t>
struct UintPtrPairHash {
    std::size_t operator()(const std::pair<std::uintptr_t, std::uintptr_t>& p) const noexcept {
        std::size_t h1 = std::hash<std::uintptr_t>{}(p.first);
        std::size_t h2 = std::hash<std::uintptr_t>{}(p.second);
        // Combine hashes using a simple but effective method
        return h1 ^ (h2 << 1);
    }
};