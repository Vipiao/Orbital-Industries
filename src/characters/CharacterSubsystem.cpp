// CharacterSubsystem.cpp
#include "CharacterSubsystem.h"
#include "DigitbotResources.h"
#include "DigitbotPlayerController.h"
#include <algorithm>

CharacterSubsystem::CharacterSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics,
                                     JobManager* jobManager, TimeHandler* timeHandler)
    : m_physics(physics)
    , m_graphics(graphics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
{
    // Initialize shared resources for Digitbot characters
    m_digitbotResources = std::make_unique<DigitbotResources>(m_graphics);

    // Create player controller
    m_playerController = std::make_unique<DigitbotPlayerController>(m_graphics);
}

CharacterSubsystem::~CharacterSubsystem() {
    // Clear all characters (will trigger Character destructors)
    m_characters.clear();
}

std::weak_ptr<Digitbot> CharacterSubsystem::createDigitbot() {
    auto digitbot = std::make_shared<Digitbot>(
        m_physics,
        m_graphics,
        m_jobManager,
        m_timeHandler,
        m_digitbotResources.get()
    );

    std::weak_ptr<Digitbot> digitbotWeak = digitbot;
    m_characters.push_back(std::move(digitbot));

    // Set as the pilotable character if we don't already have one
    if (m_playerController && m_playerController->getPilotableCharacter().expired()) {
        m_playerController->setPilotableCharacter(digitbotWeak);
    }

    return digitbotWeak;
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