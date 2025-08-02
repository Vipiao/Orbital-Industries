// PairCache.h
#pragma once

#include <unordered_map>
#include <utility>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <iostream>
#include "HashFunctions.h"

/**
 * @brief Generic global cache for storing data between pairs of objects with LRU eviction
 * 
 * Uses a global cache per DataType to ensure data is shared between all object pairs.
 * Template parameter DataType is the type of data to cache between object pairs.
 * Implements LRU eviction when cache size exceeds MAX_CACHE_SIZE.
 */
template<typename DataType>
class PairCache {
public:
    PairCache() = default;
    virtual ~PairCache() = default;
    
    /**
     * @brief Get const reference to cached data for a pair of objects from global cache
     * @param objA First object pointer (used as cache key)
     * @param objB Second object pointer (used as cache key)
     * @return Pointer to cached data, or nullptr if not found
     */
    static const DataType* getCachedData(int idA, int idB) {
        auto cacheKey = makeCacheKey(idA, idB);
        auto it = s_globalCache.find(cacheKey);
        
        if (it == s_globalCache.end()) {
            return nullptr; // No cached entry
        }
        
        const CachedInfo& info = it->second;
        
        // Update access order for LRU
        const_cast<CachedInfo&>(info).accessOrder = ++s_accessCounter;
        
        return &info.data;
    }
    
    /**
     * @brief Set cached data for a pair of objects in global cache
     * @param idA First object ID (used as cache key)
     * @param idB Second object ID (used as cache key)
     * @param data Data to cache
     */
    static void setCachedData(int idA, int idB, DataType data) {
        // Check if we need to evict old entries
        if (s_globalCache.size() >= MAX_CACHE_SIZE) {
            evictOldestCacheEntries();
        }
        
        auto cacheKey = makeCacheKey(idA, idB);
        
        CachedInfo info;
        info.data = std::move(data);
        info.accessOrder = ++s_accessCounter;
        
        s_globalCache[cacheKey] = info;
    }

    /**
     * @brief Clear cached data for a specific pair of objects
     * @param idA First object ID (used as cache key)
     * @param idB Second object ID (used as cache key)
     */
    static void clearCachedData(int idA, int idB) {
        auto cacheKey = makeCacheKey(idA, idB);
        s_globalCache.erase(cacheKey);
    }

private:
    struct CachedInfo {
        DataType data;           // The cached data
        uint64_t accessOrder;    // For LRU eviction
    };
    
    static std::pair<int, int> makeCacheKey(int idA, int idB) {
        // Sort to ensure consistent ordering
        return (idA < idB) ? std::make_pair(idA, idB) : std::make_pair(idB, idA);
    }
    
    static void evictOldestCacheEntries() {
        std::cout << "Warning: PairCache eviction triggered for cache size " 
                  << s_globalCache.size() << " (max: " << MAX_CACHE_SIZE << ")" << std::endl;
        if (s_globalCache.size() < EVICT_COUNT) {
            s_globalCache.clear();
            return;
        }
        
        // Find oldest entries by access order
        std::vector<std::pair<std::pair<int, int>, uint64_t>> entries;
        for (const auto& pair : s_globalCache) {
            entries.push_back({pair.first, pair.second.accessOrder});
        }
        
        // Sort by access order (oldest first)
        std::sort(entries.begin(), entries.end(), 
            [](const auto& a, const auto& b) { return a.second < b.second; });
        
        // Remove oldest entries
        for (size_t i = 0; i < EVICT_COUNT && i < entries.size(); ++i) {
            s_globalCache.erase(entries[i].first);
        }
    }
    
    // Static access counter per template instantiation
    static uint64_t s_accessCounter;
    static std::unordered_map<std::pair<int, int>, CachedInfo, IntPairHash> s_globalCache;
    static constexpr size_t MAX_CACHE_SIZE = 40000;
    static constexpr size_t EVICT_COUNT = 100;
};

// Static member definitions
template<typename DataType>
uint64_t PairCache<DataType>::s_accessCounter = 0;

template<typename DataType>
std::unordered_map<std::pair<int, int>, typename PairCache<DataType>::CachedInfo, IntPairHash> PairCache<DataType>::s_globalCache;