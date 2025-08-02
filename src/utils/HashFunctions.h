// HashFunctions.h
#pragma once

#include <glm/glm.hpp>
#include <cstddef>
#include <utility>
#include <glm/gtc/quaternion.hpp>
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

// Hash function for glm::dvec3
struct DVec3Hash {
    size_t operator()(const glm::dvec3& vec) const {
        size_t hash = std::hash<double>{}(vec.x);
        hash ^= std::hash<double>{}(vec.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<double>{}(vec.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
};

// Hash function for glm::dquat
struct DQuatHash {
    size_t operator()(const glm::dquat& quat) const {
        size_t hash = std::hash<double>{}(quat.w);
        hash ^= std::hash<double>{}(quat.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<double>{}(quat.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<double>{}(quat.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
};

// General hash combining function
inline size_t combineHashes(size_t hash1, size_t hash2) {
    return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
};