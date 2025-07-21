// CollisionDetector.h
#pragma once

#include "Collider.h"
#include "Edge.h"
#include "CollisionResult.h"
#include <vector>
#include <set>
#include <memory>
#include <chrono>
#include "../utils/Generator.h"

// Forward declaration
class TimeHandler;

class CollisionDetector {
public:
    CollisionDetector(TimeHandler* timeHandler);
    ~CollisionDetector() = default;
    
    // Add a collider to the collision detection system
    void addCollider(Collider* collider);
    
    // Remove a collider from the collision detection system
    void removeCollider(Collider* collider);
    
    // Run collision detection
    Generator<bool> run(std::vector<CollisionResult>& collisions);
    
    // Set the end time for the current collision detection run
    void setEndTime(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    
private:
    std::vector<Collider*> colliders;
    std::vector<std::unique_ptr<Edge>> edgesX;
    std::vector<std::unique_ptr<Edge>> edgesY;
    std::vector<std::unique_ptr<Edge>> edgesZ;

    // Active collision pairs
    std::set<std::pair<Collider*, Collider*>> m_activeAABBS;

    TimeHandler* m_timeHandler;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_endTime;

    // Helper functions
    void updateAllCollidersAndAABB();
    void updateAllEdgeValues();
    void sortAndDetectPotentialCollisions(
        std::vector<std::unique_ptr<Edge>>& edges,
        std::set<std::pair<Collider*, Collider*>>& potentialCollisions);
    void checkCollision(Collider* collider1, Collider* collider2, std::vector<CollisionResult>& collisions);
    void insertionSort(
        std::vector<std::unique_ptr<Edge>>& edges,
        std::set<std::pair<Collider*, Collider*>>& potentialCollisions);
    std::pair<Collider*, Collider*> makePair(Collider* a, Collider* b);
};