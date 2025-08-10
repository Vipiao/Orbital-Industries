#pragma once

/**
 * @file MeshHandler.h
 * 
 * @brief A mesh management system for OpenGL rendering.
 * 
 * The MeshHandler class provides a robust system for managing 3D mesh data in an OpenGL context.
 * It handles the creation, modification, and rendering of triangle-based meshes with support
 * for positions, normals, texture coordinates, and occlusion factors.
 * 
 * Key features:
 * - Efficient mesh management with unique IDs for both meshes and triangles
 * - Dynamic addition and removal of triangles/meshes
 * - Support for texture mapping
 * - Single-pass rendering with occlusion control
 * - GPU memory management with vertex buffer objects (VBOs) and vertex array objects (VAOs)
 * 
 * Usage example:
 * @code
 * // Create a mesh handler with capacity for 1000 triangles and 10 meshes
 * MeshHandler meshHandler(1000, 10);
 * 
 * // Create a new mesh
 * int meshId = meshHandler.addMesh();
 * 
 * // Define a simple triangle
 * std::vector<glm::dvec3> vertices = {
 *     glm::dvec3(-1.0, -1.0, 0.0),
 *     glm::dvec3( 1.0, -1.0, 0.0),
 *     glm::dvec3( 0.0,  1.0, 0.0)
 * };
 * std::vector<glm::dvec3> normals = {
 *     glm::dvec3(0.0, 0.0, 1.0),
 *     glm::dvec3(0.0, 0.0, 1.0),
 *     glm::dvec3(0.0, 0.0, 1.0)
 * };
 * std::vector<glm::dvec3> tangents = {
 *     glm::dvec3(1.0, 0.0, 0.0),
 *     glm::dvec3(1.0, 0.0, 0.0),
 *     glm::dvec3(1.0, 0.0, 0.0)
 * };
 * std::vector<glm::dvec2> uvs = {
 *     glm::dvec2(0.0, 0.0),
 *     glm::dvec2(1.0, 0.0),
 *     glm::dvec2(0.5, 1.0)
 * };
 * 
 * // Optional occlusion factors (0.0 = fully occluded, 1.0 = no occlusion)
 * std::vector<double> occlusionFactors = {
 *     1.0, 1.0, 1.0  // No occlusion for any vertex
 * };
 * 
 * // Add triangle to mesh and get its ID
 * std::vector<uint32_t> triangleIds = meshHandler.appendTrianglesToMesh(
 *     meshId, &vertices, &normals, &tangents, &uvs, &occlusionFactors);
 * 
 * // Update mesh position in world space
 * glm::dvec3 position(0.0, 0.0, 0.0);
 * glm::dvec3 velocity(0.0, 0.0, 0.0);
 * glm::dquat orientation(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
 * glm::dvec3 angVelAxis(0.0, 1.0, 0.0);
 * double angVel = 0.0;
 * glm::dvec3 centerOfRotation(0.0, 0.0, 0.0);
 * 
 * meshHandler.updateMeshData(
 *     meshId, &position, &velocity, orientation, 
 *     angVelAxis, angVel, centerOfRotation,
 *     -1, -1, 0);  // No textures, time = 0
 * 
 * // In render loop:
 * glm::mat4 view = camera.getViewMatrix();
 * glm::mat4 projection = camera.getProjectionMatrix();
 * glm::dvec3 lightPos(5.0, 5.0, 5.0);
 * glm::dvec3 camPos = camera.getPosition();
 * 
 * // Single pass rendering
 * meshHandler.render(view, projection, frameCount, timeMs, 
 *                   timeFraction, lightPos, camPos);
 * @endcode
 */

#include "ShaderProgram.h"

#include <glm/glm.hpp>
#include <vector>
#include <map>
#include <glad/glad.h>
#include <stdexcept>
#include <glm/gtc/quaternion.hpp>

#pragma pack(push, 1)
static_assert(sizeof(float)* std::size_t(8) == std::size_t(32), "float is not 32 bits.");
struct Vertex {
   glm::vec3 position;
   glm::vec3 normal;
   glm::vec3 tangent;
   glm::vec2 uv;
   float occlusionFactor;
   uint32_t meshIndex;
   uint32_t triangleId;
};
struct MeshData {
  glm::vec4 positionHigh{};        // Offset= 0, size=16 bytes. High part of Dekker position
  glm::vec4 positionLow{};         // Offset=16, size=16 bytes. Low part of Dekker position
   glm::vec4 velocity{};            // Offset=32, size=16 bytes.
   glm::vec4 orientation{};         // Offset=48, size=16 bytes. Quaternion
   glm::vec4 angVel{};              // Offset=64, size=16 bytes. Unit axis (xyz)
   glm::vec4 centerOfRotation{};    // Offset=80, size=16 bytes.
   glm::vec4 scale{};               // Offset=96, size=16 bytes. (xyz = scale, w = padding)
   uint32_t time{};                 // Offset=112, size= 4 bytes.
   int32_t colorTextureUnit{};      // Offset=116, size= 4 bytes. (-1 means no textures)
   int32_t normalTextureUnit{};     // Offset=120, size= 4 bytes. (-1 means no textures)
   uint32_t padding2{};             // Offset=124, size= 4 bytes. (padding)
}; // Make sure to pad so size is divisible by 16 because you have a vec4.
#pragma pack(pop)

struct MeshInfo {
   int numTriangles{ 0 };
   std::map<uint64_t, uint64_t> triangleIndices{}; // Triangle id to index in vertex data.
   int nextTriangleId{ 0 };
};

class MeshHandler {
public:
   class Texture {
   protected:
   public:
      Texture() : m_texture(0), m_textureUnit(0) {}
      Texture(unsigned int texture, unsigned int textureUnit) {
         m_texture = texture;
         m_textureUnit = textureUnit;
      }
      unsigned int m_texture{};
      unsigned int m_textureUnit{};
   };

   explicit MeshHandler(size_t maxTriangles, size_t maxMeshes);
   ~MeshHandler();

   int addMesh();
   std::vector<uint32_t> appendTrianglesToMesh(
      int meshIndex, const std::vector<glm::dvec3>* vertices,
      const std::vector<glm::dvec3>* normals,
      const std::vector<glm::dvec3>* tangents,
      const std::vector<glm::dvec2>* uvs,
      const std::vector<double>* occlusionFactors = nullptr
   );
   // removeTrianglesFromMesh: "triangleIndices" are the indices of the triangles you want to delete.
   // Does not need to be ordered
   void removeTrianglesFromMesh(int meshIndex, const std::vector<uint32_t>* triangleIds);
   void updateTrianglesInformation(
      int meshIndex,
      const std::vector<uint32_t>* triangleIds = nullptr,
      const std::vector<glm::dvec3>* normals = nullptr,
      const std::vector<glm::dvec3>* tangents = nullptr,
      const std::vector<glm::dvec2>* uvs = nullptr,
      const std::vector<double>* occlusionFactors = nullptr
   );
   void removeMesh(int meshIndex);
   void render(
      const glm::mat4& view, const glm::mat4& projection, uint64_t frame, uint64_t time,
      double timeRemainder, const glm::dvec3& lightPos, glm::dvec3 camPos);
   void updateMeshData(
      int meshIndex, const glm::dvec3* position, const glm::dvec3* velocity, glm::dquat orientation,
      glm::dvec3 angVelAxis, double angVel, glm::dvec3 centerOfRotation,
      glm::dvec3 scale,
      int32_t colorTextureUnit,
      int32_t normalTextureUnit,
      uint64_t time);
   Texture createTexture(std::string texturePath);
   void unitTest();
   ShaderProgram m_shaderProgram{};

protected:

   std::vector<Texture> m_textures{};
   unsigned int m_vertexBuffer{};
   GLuint m_meshDataBuffer{};
   unsigned int m_vao{};
   int m_totalTriangles{ 0 };
   size_t m_maxTriangles{};
   size_t m_maxMeshes{};
   size_t m_maxTextures{ 16 };
   std::vector<Vertex> m_vertexData;
   int m_nextMeshId{ 0 };
   std::map<int64_t, MeshInfo> m_meshIndexToMeshInfo;

   int getNextMeshIndex();
};