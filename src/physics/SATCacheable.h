// SATCacheable.h
#pragma once

#include <glm/glm.hpp>
#include "../utils/HashFunctions.h"
#include <unordered_map>
#include <utility>
#include <cstdint>

class SATCacheable {
public:
    virtual ~SATCacheable() = default;
    
    // Cache management for separating axes
    bool getCachedAxis(const SATCacheable* other, glm::dvec3& axis) const;
    void setCachedAxis(const SATCacheable* other, const glm::dvec3& axis);

private:
    struct CachedAxisInfo {
        glm::dvec3 axis;         // The separating axis vector
        uint64_t accessOrder;    // For LRU eviction
    };
    
    // Cache management helpers
    static std::pair<uintptr_t, uintptr_t> makeCacheKey(const SATCacheable* a, const SATCacheable* b);
    void evictOldestCacheEntries();
    
    // Static access counter (global "time")
    static uint64_t s_accessCounter;
    static constexpr size_t MAX_CACHE_SIZE = 40;
    static constexpr size_t EVICT_COUNT = 10;
    
    mutable std::unordered_map<std::pair<uintptr_t, uintptr_t>, CachedAxisInfo, UintPtrPairHash> m_separatingAxisCache;
};