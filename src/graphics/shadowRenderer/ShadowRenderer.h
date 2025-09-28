#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <utility>
#include <glad/glad.h>

class ShadowRenderer {
public:
    ShadowRenderer();
    ~ShadowRenderer();
    
    // Shadow map management
    void setupShadowMap(unsigned int width, unsigned int height);
    void resizeShadowMap(unsigned int width, unsigned int height);
    
    // Shadow pass rendering
    void beginShadowPass();
    void endShadowPass();
    
    // Getters for shadow map data
    unsigned int getShadowMapTexture() const { return m_shadowDepthTexture; }
    unsigned int getShadowMapWidth() const { return m_shadowMapWidth; }
    unsigned int getShadowMapHeight() const { return m_shadowMapHeight; }

    // Shader reloading
    std::pair<bool, std::string> reloadShaders();
    
private:
    // Shadow map resources
    unsigned int m_shadowMapFBO{};
    unsigned int m_shadowDepthTexture{};
    unsigned int m_shadowMapWidth{};
    unsigned int m_shadowMapHeight{};
    bool m_shadowMapInitialized{false};
    
    // Private methods
    void cleanupShadowMap();
    
    // Prevent copying
    ShadowRenderer(const ShadowRenderer&) = delete;
    ShadowRenderer& operator=(const ShadowRenderer&) = delete;
};
