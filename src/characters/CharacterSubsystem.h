// CharacterSubsystem.h
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

class Character;
class Digibot;
class DigibotResources;
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
                      JobManager* jobManager, TimeHandler* timeHandler,
                      int colliderClassificationJobPriority);
    ~CharacterSubsystem();

    // Character lifecycle
    std::weak_ptr<Digibot> createDigibot();
    void removeCharacter(std::weak_ptr<Character> characterWeak);

    // Updates (called by GameBase)
    void framePreRenderAll(uint64_t frameNum, double timeRemainder);
    // Once per physics step, before integration
    void stepControlAll();
    // Once per physics step, after integration; publishes state to graphics
    void stepUpdateGraphicsAll();

    // Access
    const std::vector<std::shared_ptr<Character>>& getCharacters() const { return m_characters; }

    /**
     * @brief Resolve a character by its unique id (O(1) hash lookup)
     * @return The character, or empty if no character has that id
     */
    std::weak_ptr<Character> getCharacterById(int id) const;

private:
    // Dependencies
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;

    // Job priority for character collider cell classification (supplied by the game layer)
    int m_colliderClassificationJobPriority;

    // Shared resources for character types
    std::unique_ptr<DigibotResources> m_digibotResources;

    // Character ownership
    std::vector<std::shared_ptr<Character>> m_characters;

    // Fast lookup: unique id -> Character, maintained by create/removeCharacter
    std::unordered_map<int, std::weak_ptr<Character>> m_idToCharacter;
};