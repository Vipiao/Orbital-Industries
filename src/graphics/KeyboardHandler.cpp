





#include "KeyboardHandler.h"
#include <algorithm>
#include <iostream>

#ifdef PLATFORM_LINUX
#include <GLFW/glfw3native.h>
#endif

// Initialize static members
std::vector<KeyboardHandler*> KeyboardHandler::s_allHandlers;
bool KeyboardHandler::s_callbackRegistered = false;

KeyboardHandler::KeyboardHandler(GLFWwindow* window, Mode mode, const std::filesystem::path& filepath)
   : m_mode(mode) {
   if (mode != Mode::NONE) {
      // Ensure the directory exists.
      if (!std::filesystem::exists(filepath.parent_path())) {
         std::filesystem::create_directories(filepath.parent_path());
      }

      // In RECORD mode, delete the file if it exists.
      if (mode == Mode::RECORD && std::filesystem::exists(filepath)) {
         std::filesystem::remove(filepath);
      }

      // Open the file in the appropriate mode.
      m_file.open(filepath, (mode == Mode::RECORD ? std::ios::out : std::ios::in) | std::ios::binary);
      if (!m_file.is_open()) {
         throw std::runtime_error("Failed to open file: " + filepath.string());
      }
   }
   //
   m_window = window;
   m_buttons.push_back(&m_q);
   m_buttons.push_back(&m_w);
   m_buttons.push_back(&m_e);
   m_buttons.push_back(&m_r);
   m_buttons.push_back(&m_t);
   m_buttons.push_back(&m_y);
   m_buttons.push_back(&m_u);
   m_buttons.push_back(&m_i);
   m_buttons.push_back(&m_o);
   m_buttons.push_back(&m_p);
   m_buttons.push_back(&m_a);
   m_buttons.push_back(&m_s);
   m_buttons.push_back(&m_d);
   m_buttons.push_back(&m_f);
   m_buttons.push_back(&m_g);
   m_buttons.push_back(&m_h);
   m_buttons.push_back(&m_j);
   m_buttons.push_back(&m_k);
   m_buttons.push_back(&m_l);
   m_buttons.push_back(&m_z);
   m_buttons.push_back(&m_x);
   m_buttons.push_back(&m_c);
   m_buttons.push_back(&m_v);
   m_buttons.push_back(&m_b);
   m_buttons.push_back(&m_n);
   m_buttons.push_back(&m_m);

   m_buttons.push_back(&m_lShift);
   m_buttons.push_back(&m_rShift);
   m_buttons.push_back(&m_lCtrl);
   m_buttons.push_back(&m_rCtrl);
   m_buttons.push_back(&m_space);

   m_buttons.push_back(&m_right);
   m_buttons.push_back(&m_left);
   m_buttons.push_back(&m_up);
   m_buttons.push_back(&m_down);

   m_buttons.push_back(&m_esc);

   // Register this handler in the static list
   s_allHandlers.push_back(this);
   
   // Register callback once for all handlers
   if (!s_callbackRegistered) {
      glfwSetKeyCallback(m_window, capsLockCallback);
      s_callbackRegistered = true;
   }
}

KeyboardHandler::~KeyboardHandler() {
   if (m_mode != Mode::NONE) {
      m_file.close();
   }
   
   // Remove this handler from the static list
   auto it = std::find(s_allHandlers.begin(), s_allHandlers.end(), this);
   if (it != s_allHandlers.end()) {
      s_allHandlers.erase(it);
   }
}

void KeyboardHandler::setSuppressCapsLock(bool suppress) {
   m_suppressCapsLock = suppress;
}

void KeyboardHandler::capsLockCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
   (void)scancode; // Unused
   (void)mods;     // Unused
   
   // Only handle Caps Lock press events
   if (key != GLFW_KEY_CAPS_LOCK || action != GLFW_PRESS) {
      return;
   }
   
   // Check if any handler for this window has suppression enabled
   bool shouldSuppress = false;
   for (KeyboardHandler* handler : s_allHandlers) {
      if (handler->m_window == window && handler->m_suppressCapsLock) {
         shouldSuppress = true;
         break;
      }
   }
   
   // If any handler wants to suppress, toggle Caps Lock back
   if (shouldSuppress) {
      toggleCapsLock();
   }
}

void KeyboardHandler::update() {
   for (size_t ii = 0; ii < m_buttons.size(); ii++) {
      Button* bb{ m_buttons[ii] };
      bb->m_isDown = glfwGetKey(m_window, bb->m_keyCode) == GLFW_PRESS;
      if (bb->m_isDownPrevious) {
         if (bb->m_isDown) {
            bb->m_timeDown++;
         } else {
            bb->m_timeUp = 0;
         }
      } else {
         if (bb->m_isDown) {
            bb->m_timeDown = 0;
         } else {
            bb->m_timeUp++;
         }
      }
      bb->m_isDownPrevious = bb->m_isDown;
   }
}

void KeyboardHandler::toggleCapsLock() {
#ifdef PLATFORM_WINDOWS
   // Check if the physical Caps Lock key is currently pressed
   // GetAsyncKeyState returns the key's state: high-order bit is 1 if key is down
   bool keyIsPhysicallyDown = (GetAsyncKeyState(VK_CAPITAL) & 0x8000) != 0;
   
   if (keyIsPhysicallyDown) {
      // Key is down - simulate UP first, then DOWN to toggle
      // This prevents conflict with the physical key state
      keybd_event(VK_CAPITAL, 0x45, KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP, 0);
      keybd_event(VK_CAPITAL, 0x45, KEYEVENTF_EXTENDEDKEY, 0);
   } else {
      // Key is up - normal DOWN then UP sequence
      keybd_event(VK_CAPITAL, 0x45, KEYEVENTF_EXTENDEDKEY, 0);
      keybd_event(VK_CAPITAL, 0x45, KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP, 0);
   }
#elif defined(PLATFORM_LINUX)
   Display* display = glfwGetX11Display();
   if (!display) {
      return;
   }
   
   // Get current state
   unsigned int state = 0;
   XkbGetIndicatorState(display, XkbUseCoreKbd, &state);
   bool currentState = (state & 0x01) != 0;
   
   // Toggle to opposite state
   unsigned int mask = LockMask;
   if (currentState) {
      XkbLockModifiers(display, XkbUseCoreKbd, mask, 0);
   } else {
      XkbLockModifiers(display, XkbUseCoreKbd, mask, mask);
   }
   XFlush(display);
#else
   // Platform not supported - do nothing
#endif
}