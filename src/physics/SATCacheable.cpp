// SATCacheable.cpp
#include "SATCacheable.h"
#include <algorithm>
#include <vector>

uint64_t SATCacheable::s_accessCounter = 0;

bool SATCacheable::getCachedAxis(const SATCacheable* other, glm::dvec3& axis) const {
    auto cacheKey = makeCacheKey(this, other);
    auto it = m_separatingAxisCache.find(cacheKey);
    
    if (it == m_separatingAxisCache.end()) {
        return false; // No cached entry
    }
    
    const CachedAxisInfo& info = it->second;
    
    // Update access order for LRU
    const_cast<CachedAxisInfo&>(info).accessOrder = ++s_accessCounter;
    
    axis = info.axis;
    return true;
}

void SATCacheable::setCachedAxis(const SATCacheable* other, const glm::dvec3& axis) {
    // Check if we need to evict old entries
    if (m_separatingAxisCache.size() >= MAX_CACHE_SIZE) {
        evictOldestCacheEntries();
    }
    
    auto cacheKey = makeCacheKey(this, other);
    
    CachedAxisInfo info;
    info.axis = axis;
    info.accessOrder = ++s_accessCounter;
    
    m_separatingAxisCache[cacheKey] = info;
}

std::pair<uintptr_t, uintptr_t> SATCacheable::makeCacheKey(const SATCacheable* a, const SATCacheable* b) {
    uintptr_t ptrA = reinterpret_cast<uintptr_t>(a);
    uintptr_t ptrB = reinterpret_cast<uintptr_t>(b);
    
    // Sort to ensure consistent ordering
    if (ptrA < ptrB) {
        return {ptrA, ptrB};
    } else {
        return {ptrB, ptrA};
    }
}

void SATCacheable::evictOldestCacheEntries() {
    if (m_separatingAxisCache.size() < EVICT_COUNT) {
        m_separatingAxisCache.clear();
        return;
    }
    
    // Find oldest entries by access order
    std::vector<std::pair<std::pair<uintptr_t, uintptr_t>, uint64_t>> entries;
    for (const auto& pair : m_separatingAxisCache) {
        entries.push_back({pair.first, pair.second.accessOrder});
    }
    
    // Sort by access order (oldest first)
    std::sort(entries.begin(), entries.end(), 
        [](const auto& a, const auto& b) { return a.second < b.second; });
    
    // Remove oldest entries
    for (size_t i = 0; i < EVICT_COUNT && i < entries.size(); ++i) {
        m_separatingAxisCache.erase(entries[i].first);
    }
}