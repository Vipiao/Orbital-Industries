// main.cpp
#include "src/game_base/GameBase.h"
#include "src/graphics/GraphicsEngineBase.h"
#include "src/utils/TimeHandler.h"
#include "src/debug/DebugVisualization.h"
#include "src/game_base/Creative.h"
#include "src/debug/DebugRenderer.h"
#include "src/debug/DebugGlobals.h"
#include "src/graphics/MeshManager2D.h"
#include "src/graphics/CallbackManager.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

int debug1 = 0;
int debug2 = 0;

// Define the global debug renderer (must be in exactly one .cpp file)
DebugRenderer* DebugGlobals::g_debugRenderer = nullptr;
IHashable* DebugGlobals::g_gameBase = nullptr;

class Game : public IGraphicsCallbacks, public CallbackManager, public GameBase::Callback {
private:
    std::unique_ptr<GameBase> m_gameBase;
    std::unique_ptr<DebugVisualization> m_debugViz;
    std::unique_ptr<Mode> m_mode;
    DebugRendererGuard m_debugGuard;

public:

    Game(TimeHandler* timeHandler, 
         GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE) {
        
        // Create the game base instance
        m_gameBase = std::make_unique<GameBase>(800, 600, "3D Grid Demo", timeHandler, controlMode);

        // Create crosshair using 2D mesh manager from graphics engine
        auto geometryData = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/00_crosshair.png", -1, true);
        auto crossHair = geometryData.lock() ? geometryData.lock()->createInstance() : std::weak_ptr<GeometryInstance>();
        if (auto instance = crossHair.lock()) {
            instance->setPosition(glm::vec2(0.0f, 0.0f));
            instance->setScale(glm::vec2(0.05f, 0.05f));
        }

        // Register ourselves with GameBase
        m_gameBase->addCallback(this);  // Graphics callback registration

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
        RigidBody* bb = initialGrid->getRigidBody();
        bb->m_position = {0,0,0};
        //bb->m_velocity = {0.0,0.0,-0.01};
        initialGrid->addCell(glm::ivec3(0,0,0));
        //initialGrid->addCell(glm::ivec3(0,0,0));
        //addGridBlock(initialGrid, 1, 0, 0);
        //addGridBlock(initialGrid, 2, 0, 0);
        //addGridBlock(initialGrid, 3, 0, 0);
        
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
        //int size{ 70 };
        //for (int ii = -size; ii < size; ii++)
        //{
        //    for (int jj = -size; jj < size; jj++)
        //    {
        //        for (int kk = -3; kk < -2; kk++)
        //        {
        //            initialGrid->addCell(glm::ivec3(ii, jj, kk));
        //            std::cout << ii << std::endl;
        //        }
        //    }
        //}
        
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

    void run() {
        m_gameBase->run();
    }

    // GameBase::Callback implementation
    virtual void onPhysicsUpdateComplete() override {
        m_mode->physics();
    }

    // Helper method for setting up debug visualization
    void setupDebugVisualization() {
        m_debugViz = std::make_unique<DebugVisualization>(
            m_gameBase->m_graphicsEngine->m_meshHandler.get(), m_gameBase->m_graphicsEngine->m_ssboManager.get());
        m_gameBase->setDebugRenderer(m_debugViz.get());
    }

    // IGraphicsCallbacks implementation
    virtual void preRenderCallback(uint64_t frameNum) override {
        // Call registered callbacks first
        callPreRenderCallbacks(frameNum);
        
        // Game's own preRender logic
        // Process input BEFORE calling gamebase preRenderCallback
        m_mode->processInputs();
    }

    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override {
        // Call registered callbacks first
        callRenderCallbacks(viewMatrix, projectionMatrix);
        
        // Game's own render logic
    }

    virtual void framebufferSizeCallback(int width, int height) override {
        // Call registered callbacks first
        callFramebufferSizeCallbacks(width, height);
        
        // Game's own framebuffer logic (none needed currently)
        //
    }

    virtual void windowPosCallback(int xpos, int ypos) override {
        // Call registered callbacks first
        callWindowPosCallbacks(xpos, ypos);
        
        // Game's own window position logic (none needed currently)
        //
    }

private:
};

int main() {
    try {
        // Create the TimeHandler with appropriate mode
        TimeHandler* timeHandler = new TimeHandler(TimeHandler::Mode::NONE);

        // Use existing GraphicsEngineBase::Mode for controls
        GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE;

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