// CharacterSubsystem.cpp
#include "CharacterSubsystem.h"
#include "Character.h"
#include "digibot/Digibot.h"
#include "digibot/DigibotResources.h"
#include <algorithm>
#include <cassert>

CharacterSubsystem::CharacterSubsystem(PhysicsEngine* physics, GraphicsEngine* graphics,
                                     JobManager* jobManager, TimeHandler* timeHandler,
                                     int colliderClassificationJobPriority)
    : m_physics(physics)
    , m_graphics(graphics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
    , m_colliderClassificationJobPriority(colliderClassificationJobPriority)
{
    // Initialize shared resources for Digibot characters
    m_digibotResources = std::make_unique<DigibotResources>(m_graphics);
}

CharacterSubsystem::~CharacterSubsystem() {
    // Clear mapping before destroying characters to avoid dangling entries
    m_idToCharacter.clear();

    // Clear all characters (will trigger Character destructors)
    m_characters.clear();
}

std::weak_ptr<Digibot> CharacterSubsystem::createDigibot() {
    auto digibot = std::make_shared<Digibot>(
        m_physics,
        m_graphics,
        m_jobManager,
        m_timeHandler,
        m_digibotResources.get(),
        m_colliderClassificationJobPriority
    );

    std::weak_ptr<Digibot> digibotWeak = digibot;
    m_idToCharacter[digibot->getUniqueId()] = digibot;
    m_characters.push_back(std::move(digibot));

    return digibotWeak;
}

void CharacterSubsystem::removeCharacter(std::weak_ptr<Character> characterWeak) {
    auto character = characterWeak.lock();
    if (!character) {
        return; // Character already destroyed
    }

    // Every character enters through a create call, so its id must be mapped.
    [[maybe_unused]] size_t erasedIds = m_idToCharacter.erase(character->getUniqueId());
    assert(erasedIds == 1);

    auto it = std::find_if(m_characters.begin(), m_characters.end(),
        [character](const std::shared_ptr<Character>& item) {
            return item.get() == character.get();
        });

    if (it != m_characters.end()) {
        m_characters.erase(it);
    }
}

std::weak_ptr<Character> CharacterSubsystem::getCharacterById(int id) const {
    auto it = m_idToCharacter.find(id);
    return it != m_idToCharacter.end() ? it->second : std::weak_ptr<Character>{};
}

void CharacterSubsystem::framePreRenderAll(uint64_t frameNum, double timeRemainder) {
    for (auto& character : m_characters) {
        character->framePreRender(frameNum, timeRemainder);
    }
}

void CharacterSubsystem::stepControlAll() {
    for (auto& character : m_characters) {
        character->stepControl();
    }
}

void CharacterSubsystem::stepUpdateGraphicsAll() {
    for (auto& character : m_characters) {
        character->stepUpdateGraphics();
    }
}