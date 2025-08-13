// DebugVisualization.h
#pragma once

#include "../graphics/MeshHandler.h"
#include "../graphics/SSBOManager.h"
#include "../graphics/AssimpLoader.h"
#include "../debug/DebugRenderer.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

class DebugVisualization : public DebugRenderer {
public:
    DebugVisualization(MeshHandler* meshHandler, SSBOManager* ssboManager);
    ~DebugVisualization();
    
    // Override DebugRenderer interface methods
    virtual int createSphere(const std::string& name, const glm::dvec3& position, double radius = 1.0) override;
    virtual int createSphere(const glm::dvec3& position, double radius = 1.0) override;
    virtual void removeMesh(const std::string& name) override;
    virtual void removeMesh(int id) override;
    
    // Property setters by name
    virtual void setPosition(const std::string& name, const glm::dvec3& position) override;
    virtual void setOrientation(const std::string& name, const glm::dquat& orientation) override;
    virtual void setScale(const std::string& name, const glm::dvec3& scale) override;
    
    // Property setters by ID
    virtual void setPosition(int id, const glm::dvec3& position) override;
    virtual void setOrientation(int id, const glm::dquat& orientation) override;
    virtual void setScale(int id, const glm::dvec3& scale) override;
    
    // Utility functions
    virtual int getIdFromName(const std::string& name) const override;
    virtual std::string getNameFromId(int id) const override;

    // Prefix-based operations
    virtual std::vector<int> getIdsByPrefix(const std::string& prefix) const override;
    virtual void removeMeshesByPrefix(const std::string& prefix) override;

    // Generate GeoGebra commands for visualization  
    virtual std::string generateGeogebraCommands(const std::vector<glm::dvec2>& points, int precision = 4) const override;
    virtual std::string generateGeogebraCommands(const std::vector<glm::dvec3>& points, int precision = 4) const override;

    // Update all debug shapes (call this in render loop if needed)
    void update();
    
private:
    MeshHandler* m_meshHandler;
    SSBOManager* m_ssboManager;
    std::vector<AssetMeshData> s_sphereMeshData;
    bool m_sphereMeshLoaded;
    int m_redTextureUnit;
    bool m_redTextureLoaded;
    
    // Name to ID mapping
    std::unordered_map<std::string, int> m_nameToId;
    std::unordered_map<int, std::string> m_idToName;
    
    // Mesh properties storage for updates
    struct MeshProperties {
        glm::dvec3 position{0.0};
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        glm::dvec3 scale{1.0};
    };
    std::unordered_map<int, MeshProperties> m_meshProperties;
    
    // Helper functions
    void loadSphereMeshData();
    void loadRedTexture();

    // Internal helper for updating mesh transforms
    void updateMeshTransform(int id);
};