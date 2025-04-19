#pragma once

#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "MouseHandler.h"
#include "KeyboardHandler.h"

class GraphicsEngineBase {
public:
   enum class Mode { NONE, RECORD, PLAY };
   class CallBack {
   public:
      virtual void preRenderCallback(uint64_t frameNum) = 0;
      virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) = 0;
      virtual void framebufferSizeCallback(int width, int height) = 0;
      virtual void windowPosCallback(int xpos, int ypos) = 0;
   };
   GraphicsEngineBase(Mode mode = Mode::NONE, const std::filesystem::path& filepath = "recording_path");
   ~GraphicsEngineBase();
   GraphicsEngineBase(const GraphicsEngineBase&) = delete;
   GraphicsEngineBase& operator= (const GraphicsEngineBase&) = delete;

   void setSwapInterval(int swapInterval);
   void addCallbackObject(CallBack* graphicsEngineCallback);
   void removeCallbackObject(CallBack* graphicsEngineCallback);
   void startRenderLoop();
   void setTriangleRenderMode(bool useTriangles);
   bool getTriangleRenderMode();

   GLFWwindow* m_window{ nullptr };
   unsigned int m_screen_width{ 800 };
   unsigned int m_screen_height{ 600 };
   glm::dvec3 m_camPos{ 0,0,0 };
   glm::dvec3 m_camVel{};
   int m_frameRate{ 0 };
   //glm::dquat m_camOri{ glm::sqrt(2.) / 2., -glm::sqrt(2.) / 2.,0,0 }; // 90% rotation around negative x axis.
   glm::dquat m_camOri{ 1,0,0,0 }; // Unit orientation.
   uint64_t m_frameNum{ 0 };
   double m_fieldOfView{ glm::radians(100.0) };
   // Mouse.
   MouseHandler* m_mouseHandler{ nullptr };
   KeyboardHandler* m_keyboardHandler{ nullptr };
protected:
   static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
   static void windowPosCallback(GLFWwindow* window, int xpos, int ypos);
   int getFrameRate();

   std::vector<CallBack*> m_graphicsEngineCallbacks{};
   bool m_renderTriangleMode{ false };
   bool m_windowOnTop{ false };

   glm::dvec3 m_camPosPrev{};
};

