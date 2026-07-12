// Mode.h
#pragma once

// Forward declaration
class GameBase;

/**
 * @brief Interface for different game modes (Creative, Survival, etc.)
 */
class Mode {
public:
    Mode(GameBase* gameBase) : m_gameBase(gameBase) {}
    virtual ~Mode() = default;
    
    virtual void frameProcessInputs() = 0;
    // Runs once per physics step, before integration: turns the latest input
    // into forces/commands the coming step consumes.
    virtual void stepControl() = 0;
    
protected:
    GameBase* m_gameBase;
};