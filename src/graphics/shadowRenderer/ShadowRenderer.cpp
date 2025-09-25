#include "ShadowRenderer.h"
#include <iostream>
#include <stdexcept>

ShadowRenderer::ShadowRenderer() {
    // Constructor - shadow map will be initialized when setupShadowMap is called
}

ShadowRenderer::~ShadowRenderer() {
    cleanupShadowMap();
}

void ShadowRenderer::setupShadowMap(unsigned int width, unsigned int height) {
    if (m_shadowMapInitialized) {
        cleanupShadowMap();
    }
    
    m_shadowMapWidth = width;
    m_shadowMapHeight = height;
    
    // Create framebuffer
    glGenFramebuffers(1, &m_shadowMapFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_shadowMapFBO);
    
    // Create depth texture
    glGenTextures(1, &m_shadowDepthTexture);
    glBindTexture(GL_TEXTURE_2D, m_shadowDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    
    // Set border color to maximum depth (white/far plane)
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    
    // Attach depth texture to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_shadowDepthTexture, 0);
    
    // No color buffer needed for shadow mapping
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Shadow map framebuffer not complete!");
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    m_shadowMapInitialized = true;
}

void ShadowRenderer::resizeShadowMap(unsigned int width, unsigned int height) {
    setupShadowMap(width, height);
}

void ShadowRenderer::cleanupShadowMap() {
    if (m_shadowMapInitialized) {
        glDeleteTextures(1, &m_shadowDepthTexture);
        glDeleteFramebuffers(1, &m_shadowMapFBO);
        m_shadowMapInitialized = false;
    }
}

void ShadowRenderer::beginShadowPass() {
    if (!m_shadowMapInitialized) {
        throw std::runtime_error("Shadow map not initialized. Call setupShadowMap() first.");
    }
    
    // Bind shadow map framebuffer for depth rendering
    glBindFramebuffer(GL_FRAMEBUFFER, m_shadowMapFBO);
    glViewport(0, 0, m_shadowMapWidth, m_shadowMapHeight);
    
    // Clear depth buffer
    glClear(GL_DEPTH_BUFFER_BIT);
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Optional: Enable front face culling to reduce peter panning
    // (render back faces to shadow map to push shadows away from surfaces)
    glEnable(GL_CULL_FACE);
    //glCullFace(GL_FRONT);
    glCullFace(GL_BACK);
}

void ShadowRenderer::endShadowPass() {
    // Restore default settings
    glCullFace(GL_BACK);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}