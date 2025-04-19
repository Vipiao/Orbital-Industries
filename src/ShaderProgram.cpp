
#include "ShaderProgram.h"


#include <iostream>
#include <fstream>
#include <sstream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

std::string ShaderProgram::loadTextFileFromPath(std::string path) {
   std::ifstream ifs(path);
   if (ifs.fail()) {
      std::cout << "ERROR: Could not find file at: " << path << std::endl;
      throw std::runtime_error("ERROR: Could not find file at: " + path);
   }
   std::string text((std::istreambuf_iterator<char>(ifs)),
      (std::istreambuf_iterator<char>()));
   return text;
}

void ShaderProgram::printWithLineNumbers(std::string code) {
   std::cout << "Code:\n" << std::endl;
   int lineNum{ 1 };
   std::istringstream sStream{ code };
   for (std::string line; std::getline(sStream, line); ) {
      std::string lineNumStr{ std::to_string(lineNum++) };
      std::cout << lineNumStr;
      for (size_t ii = 0; ii < 4 - lineNumStr.size(); ii++) {
         std::cout << " ";
      }
      std::cout << line << std::endl;
   }
   return;
}

void ShaderProgram::loadVertexShaderFromPath(std::string vertexCodePath) {
   std::string vertexCode(ShaderProgram::loadTextFileFromPath(vertexCodePath));
   loadVertexShader(vertexCode);
}

void ShaderProgram::loadFragmentShaderFromPath(std::string fragmentCodePath) {
   std::string fragmentCode(ShaderProgram::loadTextFileFromPath(fragmentCodePath));
   loadFragmentShader(fragmentCode);
}

void ShaderProgram::loadTessellationControlShaderFromPath(std::string tessContrCodePath) {
   std::string tessCode(ShaderProgram::loadTextFileFromPath(tessContrCodePath));
   loadTessellationControlShader(tessCode);
}

void ShaderProgram::loadTessellationEvaluationShaderFromPath(std::string tessEvalCodePath) {
   std::string tessCode(ShaderProgram::loadTextFileFromPath(tessEvalCodePath));
   loadTessellationEvaluationShader(tessCode);
}

void ShaderProgram::loadVertexShader(std::string vertexCode) {
   // vertex shader
   m_vertexShader = glCreateShader(GL_VERTEX_SHADER);
   const char* vertCode = vertexCode.c_str();
   glShaderSource(m_vertexShader, 1, &vertCode, NULL);
   glCompileShader(m_vertexShader);
   // Check for shader compile errors.
   int success;
   char infoLog[512];
   glGetShaderiv(m_vertexShader, GL_COMPILE_STATUS, &success);
   if (!success) {
      glGetShaderInfoLog(m_vertexShader, 512, NULL, infoLog);
      ShaderProgram::printWithLineNumbers(vertCode);
      std::cout << std::endl << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
      throw std::runtime_error("ERROR: SHADER::VERTEX::COMPILATION_FAILED.");
   }
   m_vertexShaderIsLoaded = true;
}

void ShaderProgram::loadFragmentShader(std::string fragmentCode) {
   // fragment shader
   m_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
   const char* fragCode = fragmentCode.c_str();
   glShaderSource(m_fragmentShader, 1, &fragCode, NULL);
   glCompileShader(m_fragmentShader);
   // Check for shader compile errors.
   int success;
   char infoLog[512];
   glGetShaderiv(m_fragmentShader, GL_COMPILE_STATUS, &success);
   if (!success) {
      glGetShaderInfoLog(m_fragmentShader, 512, NULL, infoLog);
      ShaderProgram::printWithLineNumbers(fragmentCode);
      std::cout << std::endl << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
      throw std::runtime_error("ERROR: SHADER::FRAGMENT::COMPILATION_FAILED.");
   }
   m_fragmentShaderIsLoaded = true;
}

void ShaderProgram::loadTessellationControlShader(std::string tessContrCode) {
   // Tesselation control shader.
   m_tessellationControlShader = glCreateShader(GL_TESS_CONTROL_SHADER);
   const char* tessCode = tessContrCode.c_str();
   glShaderSource(m_tessellationControlShader, 1, &tessCode, NULL);
   glCompileShader(m_tessellationControlShader);
   // Check for shader compile errors.
   int success;
   char infoLog[512];
   glGetShaderiv(m_tessellationControlShader, GL_COMPILE_STATUS, &success);
   if (!success) {
      glGetShaderInfoLog(m_tessellationControlShader, 512, NULL, infoLog);
      ShaderProgram::printWithLineNumbers(tessCode);
      std::cout << std::endl << "ERROR::SHADER::TESS_CONTR::COMPILATION_FAILED\n" << infoLog << std::endl;
      throw std::runtime_error("ERROR: SHADER::TESS_CONTR::COMPILATION_FAILED.");
   }
   m_tessellationControlShaderIsLoaded = true;
}

void ShaderProgram::loadTessellationEvaluationShader(std::string tessEvalCode) {
   // Tesselation evaluation shader.
   m_tessellationEvaluationShader = glCreateShader(GL_TESS_EVALUATION_SHADER);
   const char* tessCode = tessEvalCode.c_str();
   glShaderSource(m_tessellationEvaluationShader, 1, &tessCode, NULL);
   glCompileShader(m_tessellationEvaluationShader);
   // Check for shader compile errors.
   int success;
   char infoLog[512];
   glGetShaderiv(m_tessellationEvaluationShader, GL_COMPILE_STATUS, &success);
   if (!success) {
      glGetShaderInfoLog(m_tessellationEvaluationShader, 512, NULL, infoLog);
      ShaderProgram::printWithLineNumbers(tessCode);
      std::cout << std::endl << "ERROR::SHADER::TESS_EVAL::COMPILATION_FAILED\n" << infoLog << std::endl;
      throw std::runtime_error("ERROR: SHADER::TESS_EVAL::COMPILATION_FAILED.");
   }
   m_tessellationEvaluationShaderIsLoaded = true;
}

void ShaderProgram::linkShaders() {
   if (!m_vertexShaderIsLoaded) {
      std::cout << "ERROR: Cannot link shader program before loading vertex shader." << std::endl;
      throw std::runtime_error("ERROR: Cannot link shader program before loading vertex shader.");
   }
   if (!m_fragmentShaderIsLoaded) {
      std::cout << "ERROR: Cannot link shader program before loading fragment shader." << std::endl;
      throw std::runtime_error("ERROR: Cannot link shader program before loading fragment shader.");
   }
   //if (!m_tesselationControlShaderIsLoaded) {
   //   throw std::runtime_error (
   //      "ERROR: Cannot link shader program before loading tesselation shader."
   //   );
   //}
   //if (!m_tesselationEvaluationShaderIsLoaded) {
   //   throw std::runtime_error (
   //      "ERROR: Cannot link shader program before loading tesselation shader."
   //   );
   //}
   // link shaders
   m_shaderProgram = glCreateProgram();
   glAttachShader(m_shaderProgram, m_vertexShader);
   glAttachShader(m_shaderProgram, m_fragmentShader);
   if (m_tessellationControlShaderIsLoaded) {
      glAttachShader(m_shaderProgram, m_tessellationControlShader);
   }
   if (m_tessellationEvaluationShaderIsLoaded) {
      glAttachShader(m_shaderProgram, m_tessellationEvaluationShader);
   }
   glLinkProgram(m_shaderProgram);
   // check for linking errors
   int success;
   char infoLog[512];
   glGetProgramiv(m_shaderProgram, GL_LINK_STATUS, &success);
   if (!success) {
      glGetProgramInfoLog(m_shaderProgram, 512, NULL, infoLog);
      std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
      throw std::runtime_error("ERROR: SHADER::PROGRAM::LINKING_FAILED.");
   }
   glDeleteShader(m_vertexShader);
   glDeleteShader(m_fragmentShader);
   if (m_tessellationControlShaderIsLoaded) {
      glDeleteShader(m_tessellationControlShader);
   }
   if (m_tessellationEvaluationShaderIsLoaded) {
      glDeleteShader(m_tessellationEvaluationShader);
   }
   m_programIsLinked = true;
}

unsigned int ShaderProgram::getID() {
   if (m_programIsLinked) {
      return m_shaderProgram;
   }
   std::cout << "ERROR: Shader program is not linked." << std::endl;
   throw std::runtime_error("ERROR: Shader program is not linked.");
}

void ShaderProgram::use() {
   if (!m_programIsLinked) {
      std::cout << "ERROR: Cannot use a program before it is linked." << std::endl;
      throw std::runtime_error("ERROR: Cannot use a program before it is linked.");
   }
   glUseProgram(m_shaderProgram);
}

ShaderProgram::ShaderProgram() {
   //std::cout << std::endl << "++ Shader program" << std::endl << std::endl;
}

ShaderProgram::~ShaderProgram() {
   //std::cout << std::endl << "-- Shader program" << std::endl << std::endl;
   if (m_programIsLinked) {
      glDeleteProgram(m_shaderProgram);
   } else {
      if (m_vertexShader != 0) {
         // 0 means it is not created yet.
         glDeleteShader(m_vertexShader);
      }
      if (m_fragmentShader != 0) {
         // 0 means it is not created yet.
         glDeleteShader(m_fragmentShader);
      }
      if (m_tessellationControlShader != 0) {
         // 0 means it is not created yet.
         glDeleteShader(m_tessellationControlShader);
      }
      if (m_tessellationEvaluationShader != 0) {
         // 0 means it is not created yet.
         glDeleteShader(m_tessellationEvaluationShader);
      }
   }
}
