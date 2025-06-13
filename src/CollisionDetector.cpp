// CollisionDetector.cpp
#include "CollisionDetector.h"
#include <algorithm>
#include <iostream>

void CollisionDetector::addCollider(Collider* collider) {
    if (!collider) return;
    
    // Add to colliders list
    colliders.push_back(collider);
    
    // Create edges for this collider
    edgesX.push_back(std::make_unique<Edge>(collider, EdgeType::MIN, EdgeAxis::X));
    edgesX.push_back(std::make_unique<Edge>(collider, EdgeType::MAX, EdgeAxis::X));
    
    edgesY.push_back(std::make_unique<Edge>(collider, EdgeType::MIN, EdgeAxis::Y));
    edgesY.push_back(std::make_unique<Edge>(collider, EdgeType::MAX, EdgeAxis::Y));
    
    edgesZ.push_back(std::make_unique<Edge>(collider, EdgeType::MIN, EdgeAxis::Z));
    edgesZ.push_back(std::make_unique<Edge>(collider, EdgeType::MAX, EdgeAxis::Z));
}

void CollisionDetector::removeCollider(Collider* collider) {
    if (!collider) return;
    
    // Remove from colliders list
    colliders.erase(std::remove(colliders.begin(), colliders.end(), collider), colliders.end());
    
    // Remove edges for this collider
    auto removeEdges = [collider](std::vector<std::unique_ptr<Edge>>& edges) {
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [collider](const std::unique_ptr<Edge>& edge) {
                    return edge->collider == collider;
                }),
            edges.end()
        );
    };
    
    removeEdges(edgesX);
    removeEdges(edgesY);
    removeEdges(edgesZ);
}

void CollisionDetector::run(std::vector<CollisionResult>& collisions) {
    // Update all collision boxes
    updateAllCollisionBoxes();

    std::set<std::pair<Collider*, Collider*>> potentialCollisions;

    // Check existing active collisions and remove ones that are no longer colliding
    auto it = m_activeAABBS.begin();
    while (it != m_activeAABBS.end()) {
        if (!it->first->checkAABBCollision(it->second)) {
            it = m_activeAABBS.erase(it);
        } else {
            ++it;
        }
    }
    
    // Update all edge values
    updateAllEdgeValues();
    
    // Sort and detect collisions for each axis
    sortAndDetectPotentialCollisions(edgesX, potentialCollisions);
    sortAndDetectPotentialCollisions(edgesY, potentialCollisions);
    sortAndDetectPotentialCollisions(edgesZ, potentialCollisions);

    // Now check potential collisions that overlap on ALL axes
    for (const auto& pair : potentialCollisions) {
        if (pair.first->checkAABBCollision(pair.second)) {
            // Add to active collisions
            m_activeAABBS.insert(pair);
        }
    }
    
    // Perform detailed collision detection on all active collisions
    for (const auto& pair : m_activeAABBS) {
        checkCollision(pair.first, pair.second, collisions);
    }
}

void CollisionDetector::updateAllCollisionBoxes() {
    for (Collider* collider : colliders) {
        if (collider) {
            collider->updateCollisionBoxes();
        }
    }
}

void CollisionDetector::updateAllEdgeValues() {
    for (auto& edge : edgesX) {
        edge->updateValue();
    }
    for (auto& edge : edgesY) {
        edge->updateValue();
    }
    for (auto& edge : edgesZ) {
        edge->updateValue();
    }
}

void CollisionDetector::sortAndDetectPotentialCollisions(std::vector<std::unique_ptr<Edge>>& edges, 
                                               std::set<std::pair<Collider*, Collider*>>& potentialCollisions) {
    // Sort and collect potential collision pairs during sorting
    insertionSort(edges, potentialCollisions);
}

void CollisionDetector::insertionSort(std::vector<std::unique_ptr<Edge>>& edges, 
                                     std::set<std::pair<Collider*, Collider*>>& potentialCollisions) {
     for (size_t i = 1; i < edges.size(); ++i) {
         size_t j = i;
         while (j > 0 && edges[j]->value < edges[j-1]->value) {
            // Check if we're swapping a MAX edge past a MIN edge
            if (edges[j-1]->type == EdgeType::MAX && edges[j]->type == EdgeType::MIN) {
                // Potential collision detected
                auto pair = makePair(edges[j-1]->collider, edges[j]->collider);
                potentialCollisions.insert(pair);
            }
            
             std::swap(edges[j], edges[j-1]);
             j--;
         }
     }
 }

std::pair<Collider*, Collider*> CollisionDetector::makePair(Collider* a, Collider* b) {
    // Ensure consistent ordering to avoid duplicates
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

void CollisionDetector::checkCollision(Collider* collider1, Collider* collider2, std::vector<CollisionResult>& collisions) {
    if (!collider1 || !collider2 || collider1 == collider2) return;
    
    // Perform collision detection between the two colliders
    CollisionResult result = collider1->collideWith(collider2);
    
    if (result.hasCollision) {
        std::cout << "Collision detected between two colliders!" << std::endl;
        // Set collider references for resolution
        result.colliderA = collider1;
        result.colliderB = collider2;
        
        // Store collision for resolution
        collisions.push_back(result);
    }
}