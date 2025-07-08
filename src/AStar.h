// AStar.h
#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <algorithm>
#include <functional>

/**
 * @brief High-performance A* pathfinding algorithm using callbacks for zero-allocation neighbor processing
 * 
 * Template parameter Node must be:
 * - Copyable
 * - Hashable (std::hash<Node> must be defined)
 * - Equality comparable (operator== must be defined)
 * 
 * Uses callback-based interface for maximum performance - all lambdas are fully inlined.
 * 
 * Example usage:
 * auto result = AStar<glm::ivec3>::search(
 *     start,
 *     [&](const glm::ivec3& node) { return node == target; },
 *     [&](const glm::ivec3& node, auto callback) {
 *         // Call callback(neighbor, cost) for each valid neighbor
 *         for (auto neighbor : getValidNeighbors(node)) {
 *             callback(neighbor, getCost(node, neighbor));
 *         }
 *     },
 *     [&](const glm::ivec3& node) { return heuristic(node, target); }
 * );
 */
template<typename Node>
class AStar {
public:
    struct Result {
        bool found = false;
        std::vector<Node> path;
        double totalCost = 0.0;
    };
    
    /**
     * @brief High-performance A* search with callback-based neighbor processing
     * 
     * @param start Starting node
     * @param isTarget Function that returns true if node is the target: bool(const Node&)
     * @param getNeighbors Function that calls callback for each neighbor: void(const Node&, auto callback)
     *                     The callback signature is: void(const Node& neighbor, double cost)
     * @param heuristic Function that returns estimated cost to target: double(const Node&)
     * @return Result containing path if found, total cost, and success flag
     */
    template<typename IsTargetF, typename GetNeighborsF, typename HeuristicF>
    static Result search(
        const Node& start,
        IsTargetF isTarget,
        GetNeighborsF getNeighbors,
        HeuristicF heuristic
    );

private:
    struct NodeInfo {
        double gCost = std::numeric_limits<double>::infinity();
        double hCost = 0.0;
        double fCost() const { return gCost + hCost; }
        Node parent;
        bool hasParent = false;
    };
    
    struct OpenSetItem {
        Node node;
        double fCost;
        
        // For priority queue (max heap), we want smallest fCost first
        bool operator<(const OpenSetItem& other) const {
            return fCost > other.fCost;
        }
    };
    
    static std::vector<Node> reconstructPath(
        const std::unordered_map<Node, NodeInfo>& nodeInfo,
        const Node& target
    );
};

// Template implementation
template<typename Node>
template<typename IsTargetF, typename GetNeighborsF, typename HeuristicF>
typename AStar<Node>::Result AStar<Node>::search(
    const Node& start,
    IsTargetF isTarget,
    GetNeighborsF getNeighbors,
    HeuristicF heuristic
) {
    std::unordered_map<Node, NodeInfo> nodeInfo;
    std::unordered_set<Node> closedSet;
    std::priority_queue<OpenSetItem> openSet;
    
    // Initialize start node
    NodeInfo& startInfo = nodeInfo[start];
    startInfo.gCost = 0.0;
    startInfo.hCost = heuristic(start);
    
    openSet.push({start, startInfo.fCost()});
    
    while (!openSet.empty()) {
        OpenSetItem current = openSet.top();
        openSet.pop();
        
        // Skip if we've already processed this node
        if (closedSet.find(current.node) != closedSet.end()) {
            continue;
        }
        
        // Skip if we found a better path to this node already
        auto nodeIt = nodeInfo.find(current.node);
        if (nodeIt != nodeInfo.end() && nodeIt->second.fCost() < current.fCost) {
            continue;
        }
        
        // Move current node to closed set
        closedSet.insert(current.node);
        
        // Check if we reached the target
        if (isTarget(current.node)) {
            Result result;
            result.found = true;
            result.path = reconstructPath(nodeInfo, current.node);
            result.totalCost = nodeInfo[current.node].gCost;
            return result;
        }
        
        // Process neighbors via callback - zero allocations!
        getNeighbors(current.node, [&](const Node& neighbor, double edgeCost) {
            // Skip if neighbor is already processed
            if (closedSet.find(neighbor) != closedSet.end()) {
                return;
            }
            
            double tentativeGCost = nodeInfo[current.node].gCost + edgeCost;
            NodeInfo& neighborInfo = nodeInfo[neighbor];
            
            // If this is a better path to neighbor
            if (tentativeGCost < neighborInfo.gCost) {
                neighborInfo.gCost = tentativeGCost;
                neighborInfo.hCost = heuristic(neighbor);
                neighborInfo.parent = current.node;
                neighborInfo.hasParent = true;
                
                openSet.push({neighbor, neighborInfo.fCost()});
            }
        });
    }
    
    // No path found
    return Result{};
}

template<typename Node>
std::vector<Node> AStar<Node>::reconstructPath(
    const std::unordered_map<Node, NodeInfo>& nodeInfo,
    const Node& target
) {
    std::vector<Node> path;
    Node current = target;
    
    while (true) {
        path.push_back(current);
        
        auto it = nodeInfo.find(current);
        if (it == nodeInfo.end() || !it->second.hasParent) {
            break;
        }
        
        current = it->second.parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}