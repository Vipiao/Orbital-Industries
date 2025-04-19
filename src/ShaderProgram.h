#pragma once

#include <string>

class ShaderProgram {
private:
   unsigned int m_vertexShader{ 0 };
   unsigned int m_fragmentShader{ 0 };
   unsigned int m_tessellationControlShader{ 0 };
   unsigned int m_tessellationEvaluationShader{ 0 };
   unsigned int m_shaderProgram{ 0 };

   bool m_vertexShaderIsLoaded = false;
   bool m_fragmentShaderIsLoaded = false;
   bool m_tessellationControlShaderIsLoaded = false;
   bool m_tessellationEvaluationShaderIsLoaded = false;
   bool m_programIsLinked = false;
public:
   static std::string loadTextFileFromPath(std::string path);
   //static std::string loadTextFileFromPath(std::string path);
   static void printWithLineNumbers(std::string code);
   void loadVertexShaderFromPath(std::string vertexCodePath);
   void loadFragmentShaderFromPath(std::string fragmentCodePath);
   void loadTessellationControlShaderFromPath(std::string tessContrCodePath);
   void loadTessellationEvaluationShaderFromPath(std::string tessEvalCodePath);
   void loadVertexShader(std::string vertexCode);
   void loadFragmentShader(std::string fragmentCode);
   void loadTessellationControlShader(std::string tessContrCode);
   void loadTessellationEvaluationShader(std::string tessEvalCode);
   void linkShaders();
   unsigned int getID();
   void use();
   ShaderProgram();
   ~ShaderProgram();
};

