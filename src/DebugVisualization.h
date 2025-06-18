// DebugVisualization.h
#pragma once

#include "MeshHandler.h"
#include "AssimpLoader.h"
#include <vector>
#include <memory>

class DebugVisualization {
public:
    DebugVisualization(MeshHandler* meshHandler);
    ~DebugVisualization();
    
    // Create a debug sphere at the specified position with given radius
    int createSphere(const glm::dvec3& position, double radius = 1.0);
    
    // Delete a debug mesh by ID
    void deleteMesh(int meshId);
    
    // Update all debug shapes (call this in render loop if needed)
    void update();
    
private:
    MeshHandler* m_meshHandler;
    std::vector<AssetMeshData> s_sphereMeshData;
    bool m_sphereMeshLoaded;
    int m_redTextureUnit;
    bool m_redTextureLoaded;
    
    // List of active debug mesh IDs
    std::vector<int> m_activeMeshIds;
    
    // Helper functions
    void loadSphereMeshData();
    void loadRedTexture();
};