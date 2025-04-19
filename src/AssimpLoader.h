#pragma once

#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>

class AssimpLoader {
private:
public:
   static void load(
      std::string path,
      std::vector<std::vector<double>>* vertexData,
      std::vector<std::vector<int>>* indices,
      bool* hasTextures,
      bool ignoreTextureCoordinates
   );
};

