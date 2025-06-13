// CollisionDetector.h
#pragma once

#include "Collider.h"
#include "Edge.h"
#include <vector>
#include <set>
#include <memory>

class CollisionDetector {
public:
    CollisionDetector() = default;
    ~CollisionDetector() = default;
    
    // Add a collider to the collision detection system
    void addCollider(Collider* collider);
    
    // Remove a collider from the collision detection system
    void removeCollider(Collider* collider);
    
    // Run collision detection
    void run(std::vector<CollisionResult>& collisions);
    
private:
    std::vector<Collider*> colliders;
    std::vector<std::unique_ptr<Edge>> edgesX;
    std::vector<std::unique_ptr<Edge>> edgesY;
    std::vector<std::unique_ptr<Edge>> edgesZ;

    // Active collision pairs
    std::set<std::pair<Collider*, Collider*>> m_activeAABBS;
    
    // Helper functions
    void updateAllCollisionBoxes();
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