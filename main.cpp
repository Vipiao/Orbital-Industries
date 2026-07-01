// main.cpp
#include "src/game_base/GameBase.h"
#include "graphics/GraphicsEngineBase.h"
#include "src/utils/TimeHandler.h"
#include "src/debug/DebugVisualization.h"
#include "src/game_base/Creative.h"
#include "src/debug/DebugRenderer.h"
#include "src/debug/DebugGlobals.h"
#include "graphics/MeshManager2D/MeshManager2D.h"
#include "graphics/CallbackManager.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

int debug1 = 0;
int debug2 = 0;

// Define the global debug renderer (must be in exactly one .cpp file)
DebugRenderer* DebugGlobals::g_debugRenderer = nullptr;
IHashable* DebugGlobals::g_gameBase = nullptr;

class Game : public GameBase::Callback {
private:
    std::unique_ptr<GameBase> m_gameBase;
    std::unique_ptr<DebugVisualization> m_debugViz;
    std::unique_ptr<Mode> m_mode;
    DebugRendererGuard m_debugGuard;

    // Shader reload management
    bool m_shaderReloadRequested = false;

public:

    Game(TimeHandler* timeHandler, 
         GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::RECORD) {
        
        // Create the game base instance
        m_gameBase = std::make_unique<GameBase>(800, 600, "3D Grid Demo", timeHandler, controlMode);

        // Register physics callback
        m_gameBase->addPhysicsCallback(this);  // Physics callback registration
        
        // Setup debug visualization
        setupDebugVisualization();

        // Set global debug renderer with RAII guard  
        m_debugGuard = DebugGlobals::setDebugRenderer(m_debugViz.get());

        // Set global GameBase for debugging
        DebugGlobals::g_gameBase = m_gameBase.get();
 
        // Create creative mode
        m_mode = std::make_unique<Creative>(m_gameBase.get());

        // Set up initial camera position and orientation
        m_gameBase->m_graphicsEngine->getCamPos() = glm::dvec3(0, 0, 0);
        m_gameBase->m_graphicsEngine->getCamOri() = glm::angleAxis(glm::radians(0.0), glm::dvec3(1, 0, 0));
        m_gameBase->m_graphicsEngine->getFieldOfView() = glm::radians(90.0);
        
        // Enable mouse lock for camera control
        m_gameBase->m_graphicsEngine->getMouseHandler()->setMouseLock(true);
        
        // Create a center grid that will be our player object
        auto initialGridWeak = m_gameBase->createGrid(glm::dvec3(0, 0, 0));
        auto initialGrid = initialGridWeak.lock();
        auto rigidBodyWeak = initialGrid->getRigidBody();
        auto bb = rigidBodyWeak.lock();
        if (bb) {
            bb->m_position = {0,0,0};
            //bb->m_velocity = {0.0,0.0,-0.01};
        }
        //initialGrid->addCell(glm::ivec3(0,0,0));
        //initialGrid->addCell(glm::ivec3(1,0,0));
        //bb->setAngularVelocityBody({0,0,0.1});

        // Create a Digibot character at origin
        auto digibotWeak = m_gameBase->createDigibot();
        auto digibot = digibotWeak.lock();
        if (digibot) {
            //digibot->showCollisionBox();
        }
        //if (auto digibotRb = digibot->getRigidBody().lock()) {
        //    digibotRb->m_velocity.x += 2;
        //}
        //if (auto gridRb = initialGrid->getRigidBody().lock()) {
        //    gridRb->m_velocity.x += 2.;
        //}
        
        //for (int ll = 0; ll < 2; ll++) {
        //    for (int ii = -3; ii < 4; ii++)
        //    {
        //        for (int jj = -3; jj < 4; jj++)
        //        {
        //            
        //            for (int kk = -3; kk < 4; kk++)
        //            {
        //                initialGrid->addCell(glm::ivec3(ii + ll*10, jj, kk));
        //            }
        //        }
        //    }
        //    for (int ii = -2; ii < 3; ii++)
        //    {
        //        for (int jj = -2; jj < 3; jj++)
        //        {
        //            
        //            for (int kk = -2; kk < 3; kk++)
        //            {
        //                initialGrid->removeCell(glm::ivec3(ii + ll*10, jj, kk));
        //            }
        //        }
        //    }
        //}
        //for (int ii = 4; ii < 7; ii++)
        //{
        //    for (int jj = -1; jj < 2; jj++)
        //    {
        //        
        //        for (int kk = -2; kk < 2; kk++)
        //        {
        //            initialGrid->addCell(glm::ivec3(ii, jj, kk));
        //        }
        //    }
        //}
        //for (int ii = 4-1; ii < 7+1; ii++)
        //{
        //    for (int jj = -1+1; jj < 2-1; jj++)
        //    {
        //        
        //        for (int kk = -2+1; kk < 2-1; kk++)
        //        {
        //            initialGrid->removeCell(glm::ivec3(ii, jj, kk));
        //        }
        //    }
        //}
        // Ground.
        int size{ 10 };
        for (int ii = -size; ii < size; ii++)
        {
            for (int jj = -size; jj < size; jj++)
            {
                for (int kk = -3; kk < -2; kk++)
                {
                    initialGrid->addCell(glm::ivec3(ii, jj, kk));
                    std::cout << ii << std::endl;
                }
            }
        }
        
        // Print instructions
        std::cout << "3D Grid Block Demo" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  WASD: Move camera" << std::endl;
        std::cout << "  Mouse: Look around" << std::endl;
        std::cout << "  Space/Shift: Move up/down" << std::endl;
        std::cout << "  M: Toggle mouse lock" << std::endl;
        std::cout << "  F: Apply force to grid" << std::endl;
        std::cout << "  R: Configure block (select corners)" << std::endl;
        std::cout << "  Q: Remove block at (1,1,1)" << std::endl;
    }

    // Expose GameBase for Mode access
    GameBase* getGameBase() { return m_gameBase.get(); }

    void onFrame() {
        // Begin frame
        m_gameBase->beginFrame();
        
        // Process game-specific input
        if (m_gameBase->m_graphicsEngine->getKeyboardHandler()->m_n.justPressed()) {
            m_shaderReloadRequested = true;
        }
        m_mode->processInputs();
        
        // Render
        m_gameBase->render();
        
        // End frame
        m_gameBase->endFrame();
    }

    void run() {
        while (!m_gameBase->m_graphicsEngine->getGraphicsEngineBase()->shouldClose()) {
            onFrame();
        }
    }

    // GameBase::Callback implementation
    virtual void onPhysicsUpdateComplete() override {
        m_mode->physics();

        // Handle shader reload request if pending
        if (m_shaderReloadRequested) {
            m_shaderReloadRequested = false;
            auto [success, message] = m_gameBase->reloadShaders();
            std::cout << "Shader Reload " << (success ? "SUCCESS" : "FAILED") << ": " << message << std::endl;
        }
    }

    // Helper method for setting up debug visualization
    void setupDebugVisualization() {
        m_debugViz = std::make_unique<DebugVisualization>(
            m_gameBase->m_graphicsEngine->getInstanceHandler(), m_gameBase->m_graphicsEngine->m_ssboManager.get());
        m_gameBase->setDebugRenderer(m_debugViz.get());
    }

private:
};

int main() {
    try {
        // Create the TimeHandler with appropriate mode
        TimeHandler* timeHandler = new TimeHandler(TimeHandler::Mode::RECORD);

        // Use existing GraphicsEngineBase::Mode for controls
        GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::RECORD;

        Game game(timeHandler, controlMode);
        game.run();
        
        // Clean up TimeHandler
        delete timeHandler;
    } catch (const std::bad_alloc& e) {
        std::cerr << "Out of memory: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}