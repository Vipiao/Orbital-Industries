// Mode.h
#pragma once

#include <memory>

// Forward declarations
class GameBase;
class Digibot;

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

    // True while this mode is actively driving the player character this frame
    // (character control, as opposed to free camera).
    virtual bool isControllingCharacter() const { return false; }

    // The player is asking to control a character (e.g. the selection tool is
    // engaged). Whether control actually happens is decided by the layer above:
    // a character must be granted and bound first, so control of the same
    // character is never taken on two peers at once.
    virtual bool wantsCharacterControl() const { return false; }
    // Which character the player is asking for (e.g. the one nearest the
    // camera); empty means no preference.
    virtual std::weak_ptr<Digibot> desiredCharacter() const { return {}; }
    // Bind the granted character this mode drives; an empty pointer unbinds.
    virtual void bindCharacter(const std::weak_ptr<Digibot>&) {}
    virtual bool hasBoundCharacter() const { return false; }

protected:
    GameBase* m_gameBase;
};