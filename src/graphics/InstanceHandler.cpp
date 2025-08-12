// InstanceHandler.cpp
#include "InstanceHandler.h"
#include "ShaderProgram.h"
#include <iostream>
#include <algorithm>
#include <set>

// Static member initialization
uint64_t Geometry::s_nextGeometryId = 0;
uint64_t Instance::s_nextInstanceId = 0;

Geometry::~Geometry() {
    if (m_VAO != 0) glDeleteVertexArrays(1, &m_VAO);
    if (m_VBO != 0) glDeleteBuffers(1, &m_VBO);
    if (m_EBO != 0) glDeleteBuffers(1, &m_EBO);
    if (m_instanceVBO != 0) glDeleteBuffers(1, &m_instanceVBO);
}

std::weak_ptr<Instance> Geometry::addInstance(int32_t meshIndex, int32_t colorTextureUnit, int32_t normalTextureUnit) {
    auto instance = std::make_shared<Instance>();
    instance->m_meshIndex = meshIndex;
    instance->m_colorTextureUnit = colorTextureUnit;
    instance->m_normalTextureUnit = normalTextureUnit;
    
    // Set buffer index to end of current data
    instance->m_bufferIndex = static_cast<uint32_t>(m_instances.size());
    
    // Add to both vectors at same index
    m_instances.push_back(instance);

    InstanceData data = createInstanceData(instance.get());
    m_instanceData.push_back(data);
    
    // Check if we need to grow the buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVBO);
    if (m_instanceData.size() > m_instanceBufferCapacity) {
        // Double the buffer capacity
        m_instanceBufferCapacity = std::max(static_cast<size_t>(1), m_instanceBufferCapacity * 2);
        // Allocate larger buffer (uninitialized)
        glBufferData(GL_ARRAY_BUFFER, m_instanceBufferCapacity * sizeof(InstanceData), 
                     nullptr, GL_DYNAMIC_DRAW);
        // Copy only the valid data
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_instanceData.size() * sizeof(InstanceData), 
                       m_instanceData.data());
        std::cout << "Geometry " << m_uniqueId << ": Grew instance buffer to capacity " 
                  << m_instanceBufferCapacity << std::endl;
    } else {
        // Just update the new instance data
        glBufferSubData(GL_ARRAY_BUFFER, (m_instanceData.size() - 1) * sizeof(InstanceData), 
                       sizeof(InstanceData), &data);
    }
    
    std::cout << "Geometry " << m_uniqueId << ": Added instance " << instance->m_uniqueId 
              << " at index " << instance->m_bufferIndex << std::endl;
    
    return instance;
}

void Geometry::removeInstance(std::weak_ptr<Instance> instanceWeak) {
    auto instance = instanceWeak.lock();
    if (!instance) return;
    
    uint32_t index = instance->m_bufferIndex;
    if (index >= m_instances.size()) {
        std::cerr << "Geometry: Invalid buffer index in removeInstance" << std::endl;
        return;
    }
    
    // If not the last element, move last element to this position
    if (index != m_instances.size() - 1) {
        // Move last instance to this position in both vectors
        m_instances[index] = m_instances.back();
        m_instanceData[index] = m_instanceData.back();
        // Update moved instance's buffer index
        m_instances[index]->m_bufferIndex = index;
    }
    
    // Remove last element from both vectors
    m_instances.pop_back();
    m_instanceData.pop_back();
    
    // Update GPU buffer only if we moved an instance
    if (!m_instanceData.empty() && index != m_instanceData.size()) {
         glBindBuffer(GL_ARRAY_BUFFER, m_instanceVBO);
        glBufferSubData(GL_ARRAY_BUFFER, index * sizeof(InstanceData), 
                       sizeof(InstanceData), &m_instanceData[index]);
    }
    
    std::cout << "Geometry " << m_uniqueId << ": Removed instance " << instance->m_uniqueId << std::endl;
}

void Geometry::updateInstanceInBuffer(Instance* instance) {
    if (instance->m_bufferIndex >= m_instanceData.size()) {
        return;
    }
    
    // Update CPU data
    m_instanceData[instance->m_bufferIndex] = createInstanceData(instance);
    
    // Update GPU buffer at specific offset
    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, instance->m_bufferIndex * sizeof(InstanceData), 
                    sizeof(InstanceData), &m_instanceData[instance->m_bufferIndex]);
}

InstanceData Geometry::createInstanceData(Instance* instance) {
    InstanceData data;
    
    data.localPosition = instance->m_localPosition;
    data.padding1 = 0.0f;
    data.localOrientation = glm::vec4(instance->m_localOrientation.x, instance->m_localOrientation.y, 
                                    instance->m_localOrientation.z, instance->m_localOrientation.w);
    data.localScale = instance->m_localScale;
    data.meshIndex = instance->m_meshIndex;
    data.colorTextureUnit = instance->m_colorTextureUnit;
    data.normalTextureUnit = instance->m_normalTextureUnit;
    data.padding2 = 0;
    data.padding3 = 0;
    
    return data;
}

InstanceHandler::InstanceHandler(SSBOManager* ssboManager, uint32_t maxTextures)
    : m_ssboManager(ssboManager), m_maxTextures(maxTextures) {
    
    if (!m_ssboManager) {
        throw std::runtime_error("SSBOManager cannot be null");
    }
    
    // Create shader program (use instance-specific shaders)
    createShaderProgram();
    
    std::cout << "InstanceHandler: Created with max " << m_maxTextures << " textures" << std::endl;
}

InstanceHandler::~InstanceHandler() {
    // Clean up OpenGL resources
    if (m_shaderProgram != 0) {
        glDeleteProgram(m_shaderProgram);
    }
    
    // Clean up textures
    for (const TextureInfo& texture : m_textures) {
        glDeleteTextures(1, &texture.textureId);
    }
}

void InstanceHandler::createShaderProgram() {
    // Use instance-specific vertex shader but reuse fragment shader
    ShaderProgram shaderProgram;
    shaderProgram.loadVertexShaderFromPath("../src/graphics/instance_vertex_shader.vert");
    shaderProgram.loadFragmentShaderFromPath("../src/graphics/fragment_shader.frag");
    shaderProgram.linkShaders();
    m_shaderProgram = shaderProgram.getID();
}

int InstanceHandler::createTexture(const std::string& texturePath) {
    // Check if texture already exists
    for (size_t i = 0; i < m_textures.size(); ++i) {
        if (m_textures[i].path == texturePath) {
            // Increment reference count
            m_textures[i].refCount++;
            return static_cast<int>(i);
        }
    }
    
    // Find available texture unit
    int textureUnit = findAvailableTextureUnit();
    if (textureUnit < 0) {
        throw std::runtime_error("No available texture units");
    }
    
    // Load texture
    GLuint textureId;
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Load image data using STB (same as MeshHandler)
    int width, height, nrChannels;
    unsigned char* data = STBImageLoader::load(true, texturePath, &width, &height, &nrChannels);
    
    if (nrChannels == 3) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    } else {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    }
    glGenerateMipmap(GL_TEXTURE_2D);
    STBImageLoader::free(data);
    
    // Create texture info
    TextureInfo info;
    info.textureId = textureId;
    info.textureUnit = static_cast<uint32_t>(textureUnit);
    info.refCount = 1;
    info.path = texturePath;
    
    m_textures.push_back(info);
    int textureIndex = static_cast<int>(m_textures.size() - 1);
    
    std::cout << "InstanceHandler: Created texture " << textureIndex 
              << " (unit " << textureUnit << ") from " << texturePath << std::endl;
    
    return textureIndex;
}

void InstanceHandler::releaseTexture(int textureIndex) {
    if (textureIndex < 0 || textureIndex >= static_cast<int>(m_textures.size())) {
        std::cerr << "InstanceHandler: Warning - texture index " << textureIndex << " out of range" << std::endl;
        return;
    }
    
    TextureInfo& info = m_textures[textureIndex];
    info.refCount--;
    
    if (info.refCount == 0) {
        glDeleteTextures(1, &info.textureId);
        std::cout << "InstanceHandler: Released texture " << textureIndex << std::endl;
        
        // Mark as deleted but don't remove from vector to keep indices stable
        info.textureId = 0;
        info.path.clear();
    }
}

std::weak_ptr<Geometry> InstanceHandler::createGeometry(const std::string& modelPath) {
    auto geometry = std::make_shared<Geometry>();
    
    loadGeometryFromFile(geometry.get(), modelPath);
    m_geometries.push_back(geometry);
    
    std::cout << "InstanceHandler: Created geometry " << geometry->m_uniqueId 
              << " from " << modelPath << std::endl;
    
    return geometry;
}

void InstanceHandler::releaseGeometry(std::weak_ptr<Geometry> geometryWeak) {
    auto geometry = geometryWeak.lock();
    if (!geometry) return;
    
    // Remove geometry
    m_geometries.erase(
        std::remove_if(m_geometries.begin(), m_geometries.end(),
            [geometry](const std::shared_ptr<Geometry>& geom) {
                return geom->m_uniqueId == geometry->m_uniqueId;
            }),
        m_geometries.end()
    );
    
    std::cout << "InstanceHandler: Released geometry " << geometry->m_uniqueId << std::endl;
}

void InstanceHandler::loadGeometryFromFile(Geometry* geometry, const std::string& modelPath) {
    std::vector<AssetMeshData> meshes;
    AssimpLoader::load(modelPath, &meshes, false);
    
    if (meshes.empty()) {
        throw std::runtime_error("No meshes found in model file");
    }
    
    // Combine all meshes into single geometry
    std::vector<GeometryVertex> vertices;
    std::vector<uint32_t> indices;
    
    for (const AssetMeshData& mesh : meshes) {
        uint32_t baseVertex = static_cast<uint32_t>(vertices.size());
        
        // Add vertices
        for (size_t i = 0; i < mesh.positionsData.size(); ++i) {
            GeometryVertex vertex;
            vertex.position = glm::vec3(mesh.positionsData[i][0], mesh.positionsData[i][1], mesh.positionsData[i][2]);
            vertex.normal = glm::vec3(mesh.normalsData[i][0], mesh.normalsData[i][1], mesh.normalsData[i][2]);
            vertex.tangent = glm::vec3(mesh.tangentsData[i][0], mesh.tangentsData[i][1], mesh.tangentsData[i][2]);
            vertex.uv = glm::vec2(mesh.uvsData[i][0], mesh.uvsData[i][1]);
            vertices.push_back(vertex);
        }
        
        // Add indices
        if (!mesh.indices.empty()) {
            for (int index : mesh.indices) {
                indices.push_back(baseVertex + static_cast<uint32_t>(index));
            }
        } else {
            // Generate indices for triangle list
            for (size_t i = 0; i < mesh.positionsData.size(); ++i) {
                indices.push_back(baseVertex + static_cast<uint32_t>(i));
            }
        }
    }
    
    setupGeometryOpenGL(geometry, vertices, indices);
}

void InstanceHandler::setupGeometryOpenGL(Geometry* geometry, 
                                        const std::vector<GeometryVertex>& vertices,
                                        const std::vector<uint32_t>& indices) {
    geometry->m_vertexCount = static_cast<uint32_t>(vertices.size());
    geometry->m_indexCount = static_cast<uint32_t>(indices.size());
    geometry->m_hasIndices = !indices.empty();
    
    // Generate VAO
    glGenVertexArrays(1, &geometry->m_VAO);
    glBindVertexArray(geometry->m_VAO);
    
    // Generate and fill VBO
    glGenBuffers(1, &geometry->m_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, geometry->m_VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GeometryVertex), 
                 vertices.data(), GL_STATIC_DRAW);
    
    // Setup vertex attributes (per-vertex data)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GeometryVertex), (void*)offsetof(GeometryVertex, position));
    glEnableVertexAttribArray(0);
    
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GeometryVertex), (void*)offsetof(GeometryVertex, normal));
    glEnableVertexAttribArray(1);
    
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(GeometryVertex), (void*)offsetof(GeometryVertex, tangent));
    glEnableVertexAttribArray(2);
    
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(GeometryVertex), (void*)offsetof(GeometryVertex, uv));
    glEnableVertexAttribArray(3);
    
    // Create instance buffer
    glGenBuffers(1, &geometry->m_instanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, geometry->m_instanceVBO);
    
    // Setup instance attributes (per-instance data)
    // Local position
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)offsetof(InstanceData, localPosition));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);
    
    // Local orientation quaternion
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)offsetof(InstanceData, localOrientation));
    glEnableVertexAttribArray(5);
    glVertexAttribDivisor(5, 1);
    
    // Local scale
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)offsetof(InstanceData, localScale));
    glEnableVertexAttribArray(6);
    glVertexAttribDivisor(6, 1);
    
    // Mesh index for SSBO lookup
    glVertexAttribIPointer(7, 1, GL_INT, sizeof(InstanceData), (void*)offsetof(InstanceData, meshIndex));
    glEnableVertexAttribArray(7);
    glVertexAttribDivisor(7, 1);
    
    // Texture units
    glVertexAttribIPointer(8, 1, GL_INT, sizeof(InstanceData), (void*)offsetof(InstanceData, colorTextureUnit));
    glEnableVertexAttribArray(8);
    glVertexAttribDivisor(8, 1);
    
    glVertexAttribIPointer(9, 1, GL_INT, sizeof(InstanceData), (void*)offsetof(InstanceData, normalTextureUnit));
    glEnableVertexAttribArray(9);
    glVertexAttribDivisor(9, 1);
    
    // Generate and fill EBO if we have indices
    if (geometry->m_hasIndices) {
        glGenBuffers(1, &geometry->m_EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, geometry->m_EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), 
                     indices.data(), GL_STATIC_DRAW);
    }
    
    glBindVertexArray(0);
}

void InstanceHandler::render(const glm::mat4& view, const glm::mat4& projection, 
                           uint64_t frame, uint64_t time, double timeRemainder, 
                           const glm::dvec3& lightPos, const glm::dvec3& camPos) {
    if (m_geometries.empty()) return;
    
    glUseProgram(m_shaderProgram);
    
    // Set uniforms (same as MeshHandler)
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    GLint projectionLoc = glGetUniformLocation(m_shaderProgram, "projection");
    GLint frameLoc = glGetUniformLocation(m_shaderProgram, "u_frame");
    GLint timeLoc = glGetUniformLocation(m_shaderProgram, "u_time");
    GLint timeRemainderLoc = glGetUniformLocation(m_shaderProgram, "u_timeRemainder");
    GLint cameraPosHighLoc = glGetUniformLocation(m_shaderProgram, "u_cameraPositionHigh");
    GLint cameraPosLowLoc = glGetUniformLocation(m_shaderProgram, "u_cameraPositionLow");
    GLint lightPosLoc = glGetUniformLocation(m_shaderProgram, "u_lightPos");
    
    if (viewLoc != -1) glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    if (projectionLoc != -1) glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
    if (frameLoc != -1) glUniform1ui(frameLoc, frame);
    if (timeLoc != -1) glUniform1ui(timeLoc, time);
    if (timeRemainderLoc != -1) glUniform1f(timeRemainderLoc, static_cast<float>(timeRemainder));
    
    // Set camera position using Dekker arithmetic
    {
        DekkerArithmetic<float>::DekkerNumber camX(camPos.x);
        DekkerArithmetic<float>::DekkerNumber camY(camPos.y);
        DekkerArithmetic<float>::DekkerNumber camZ(camPos.z);
        glm::vec3 camPosHigh(camX.main, camY.main, camZ.main);
        glm::vec3 camPosLow(camX.error, camY.error, camZ.error);
        if (cameraPosHighLoc != -1) glUniform3fv(cameraPosHighLoc, 1, glm::value_ptr(camPosHigh));
        if (cameraPosLowLoc != -1) glUniform3fv(cameraPosLowLoc, 1, glm::value_ptr(camPosLow));
    }
    
    // Set light position
    if (lightPosLoc != -1) {
        glm::dvec3 lightPosL = lightPos - camPos;
        glm::vec3 lightPosFloat(lightPosL);
        glUniform3fv(lightPosLoc, 1, glm::value_ptr(lightPosFloat));
    }
    
    // Bind all textures
    for (size_t i = 0; i < m_textures.size(); ++i) {
        const TextureInfo& texture = m_textures[i];
        if (texture.textureId != 0) { // Skip deleted textures
            glActiveTexture(GL_TEXTURE0 + texture.textureUnit);
            glBindTexture(GL_TEXTURE_2D, texture.textureId);
            
            // Set texture uniform
            std::string textureName = "u_textures[" + std::to_string(texture.textureUnit) + "]";
            GLint textureLoc = glGetUniformLocation(m_shaderProgram, textureName.c_str());
            if (textureLoc != -1) {
                glUniform1i(textureLoc, static_cast<GLint>(texture.textureUnit));
            }
        }
    }
    
    // Render each geometry with its instances
    for (const auto& geometry : m_geometries) {
        if (geometry->m_instanceData.empty()) continue;
        
        glBindVertexArray(geometry->m_VAO);
        
        if (geometry->m_hasIndices) {
            glDrawElementsInstanced(GL_TRIANGLES, geometry->m_indexCount, GL_UNSIGNED_INT, 0, 
                                  static_cast<GLsizei>(geometry->m_instanceData.size()));
        } else {
            glDrawArraysInstanced(GL_TRIANGLES, 0, geometry->m_vertexCount, 
                                static_cast<GLsizei>(geometry->m_instanceData.size()));
        }
    }
    
    glBindVertexArray(0);
}

int InstanceHandler::findAvailableTextureUnit() {
    // Simple linear search for available unit
    for (uint32_t unit = 0; unit < m_maxTextures; ++unit) {
        bool unitUsed = false;
        for (const TextureInfo& texture : m_textures) {
            if (texture.textureId != 0 && texture.textureUnit == unit) {
                unitUsed = true;
                break;
            }
        }
        if (!unitUsed) {
            return static_cast<int>(unit);
        }
    }
    return -1; // No available units
}