// CharacterSubsystem.h
#pragma once

#include "Character.h"
#include "Digitbot.h"
#include "DigitbotResources.h"
#include <memory>
#include <vector>

class PhysicsEngine;
class GraphicsEngine;
class JobManager;
class TimeHandler;

/**
 * @brief Subsystem managing all character lifecycle and updates
 * 
 * Owns all Character instances and coordinates their operations with physics/graphics.
 * Parallel to GridSubsystem in architecture.
 */
class CharacterSubsystem {
public:
    CharacterSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics,
                      JobManager* jobManager, TimeHandler* timeHandler);
    ~CharacterSubsystem();

    // Character lifecycle
    std::weak_ptr<Digitbot> createDigitbot();
    void removeCharacter(std::weak_ptr<Character> characterWeak);

    // Updates (called by GameBase each frame)
    void updateAllPreRender(uint64_t frameNum);
    void updateAllPhysicsComplete();

    // Access
    const std::vector<std::shared_ptr<Character>>& getCharacters() const { return m_characters; }

private:
    // Dependencies
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;

    // Shared resources for character types
    std::unique_ptr<DigitbotResources> m_digitbotResources;

    // Character ownership
    std::vector<std::shared_ptr<Character>> m_characters;
};