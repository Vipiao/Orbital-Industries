
#include "AssimpLoader.h"

#include <vector>
#include <stdexcept>
#include <iostream>

void processNode(
   aiNode* node, const aiScene* scene,
   std::vector<std::vector<double>>* vertexData,
   std::vector<std::vector<int>>* indices,
   bool* hasTextures,
   bool ignoreTextureCoordinates
) {
   // process all the node's meshes (if any)
   for (unsigned int ii = 0; ii < node->mNumMeshes; ii++) {
      //
      vertexData->push_back({});
      std::vector<double>* newVector = &vertexData->back();
      indices->push_back({});
      std::vector<int>* newIndices = &indices->back();
      //
      aiMesh* mesh = scene->mMeshes[node->mMeshes[ii]];
      // Vertices.
      for (unsigned int jj = 0; jj < mesh->mNumVertices; jj++) {
         newVector->push_back(mesh->mVertices[jj].x);
         newVector->push_back(mesh->mVertices[jj].y);
         newVector->push_back(mesh->mVertices[jj].z);
         // Normals.
         if (mesh->HasNormals()) {
            newVector->push_back(mesh->mNormals[jj].x);
            newVector->push_back(mesh->mNormals[jj].y);
            newVector->push_back(mesh->mNormals[jj].z);
         }
         // Texture coordinates.
         if (mesh->mTextureCoords[0]) {
            if (!ignoreTextureCoordinates) {
               newVector->push_back(mesh->mTextureCoords[0][jj].x);
               newVector->push_back(1.0 - mesh->mTextureCoords[0][jj].y);
            }
         } else {
            *hasTextures = false;
         }
      }
      // Indices.
      for (unsigned int jj = 0; jj < mesh->mNumFaces; jj++) {
         aiFace face = mesh->mFaces[jj];
         for (unsigned int kk = 0; kk < face.mNumIndices; kk++)
            newIndices->push_back(face.mIndices[kk]);
      }
      // Materials.
      if (mesh->mMaterialIndex) {
         /*aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
         vector<Texture> diffuseMaps = loadMaterialTextures (material,
            aiTextureType_DIFFUSE, "texture_diffuse");
         textures.insert (textures.end (), diffuseMaps.begin (), diffuseMaps.end ());
         vector<Texture> specularMaps = loadMaterialTextures (material,
            aiTextureType_SPECULAR, "texture_specular");
         textures.insert (textures.end (), specularMaps.begin (), specularMaps.end ());*/
      }
   }
   // then do the same for each of its children
   for (unsigned int i = 0; i < node->mNumChildren; i++) {
      processNode(node->mChildren[i], scene, vertexData, indices, hasTextures, ignoreTextureCoordinates);
   }
}

void AssimpLoader::load(
   std::string path,
   std::vector<std::vector<double>>* vertexData,
   std::vector<std::vector<int>>* indices,
   bool* hasTextures,
   bool ignoreTextureCoordinates
) {
   Assimp::Importer importer;
   const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

   if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
      std::cout << "ERROR: Loading file at: \"" + path + "\" failed." << std::endl;
      throw std::runtime_error("ERROR: Loading file at: \"" + path + "\" failed.");
   }

   std::string directory = path.substr(0, path.find_last_of('/'));

   *hasTextures = true;
   processNode(scene->mRootNode, scene, vertexData, indices, hasTextures, ignoreTextureCoordinates);
}

/*Mesh processMesh (aiMesh* mesh, const aiScene* scene) {
   vector<Vertex> vertices;
   vector<unsigned int> indices;
   vector<Texture> textures;

   for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
      Vertex vertex;
      // process vertex positions, normals and texture coordinates
      [...]
      vertices.push_back (vertex);
   }
   // process indices
   [...]
   // process material
   if (mesh->mMaterialIndex >= 0) {
      [...]
   }

   return Mesh (vertices, indices, textures);
}*/
