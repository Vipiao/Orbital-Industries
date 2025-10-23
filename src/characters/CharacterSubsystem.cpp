// CharacterSubsystem.cpp
#include "CharacterSubsystem.h"
#include "digibot/DigibotResources.h"
#include <algorithm>

CharacterSubsystem::CharacterSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics,
                                     JobManager* jobManager, TimeHandler* timeHandler)
    : m_physics(physics)
    , m_graphics(graphics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
{
    // Initialize shared resources for Digibot characters
    m_digibotResources = std::make_unique<DigibotResources>(m_graphics);
}

CharacterSubsystem::~CharacterSubsystem() {
    // Clear all characters (will trigger Character destructors)
    m_characters.clear();
}

std::weak_ptr<Digibot> CharacterSubsystem::createDigibot() {
    auto digibot = std::make_shared<Digibot>(
        m_physics,
        m_graphics,
        m_jobManager,
        m_timeHandler,
        m_digibotResources.get()
    );

    std::weak_ptr<Digibot> digibotWeak = digibot;
    m_characters.push_back(std::move(digibot));

    return digibotWeak;
}

void CharacterSubsystem::removeCharacter(std::weak_ptr<Character> characterWeak) {
    auto character = characterWeak.lock();
    if (!character) {
        return; // Character already destroyed
    }

    auto it = std::find_if(m_characters.begin(), m_characters.end(),
        [character](const std::shared_ptr<Character>& item) {
            return item.get() == character.get();
        });

    if (it != m_characters.end()) {
        m_characters.erase(it);
    }
}

void CharacterSubsystem::updateAllPreRender(uint64_t frameNum) {
    for (auto& character : m_characters) {
        character->preRenderCallback(frameNum);
    }
}

void CharacterSubsystem::updateAllPhysicsComplete() {
    for (auto& character : m_characters) {
        character->onPhysicsUpdateComplete();
    }
}