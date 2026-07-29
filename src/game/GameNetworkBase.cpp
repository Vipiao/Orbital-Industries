#include "GameNetworkBase.h"

#include "../characters/Character.h"
#include "../characters/CharacterSubsystem.h"
#include "../characters/digibot/Digibot.h"
#include "../characters/digibot/DigibotController.h"
#include "../game_base/GameBase.h"
#include "../game_base/Grid.h"
#include "../game_base/GridSubsystem.h"
#include "../game_base/Mode.h"
#include "../serialization/ByteStream.h"

#include <algorithm>
#include <cassert>
#include <iostream>

GameNetworkBase::GameNetworkBase(INetworkTransport* transport, GameBase* gameBase)
    : m_transport{transport}, m_gameBase{gameBase} {
    assert(m_transport != nullptr && m_gameBase != nullptr);
}

void GameNetworkBase::broadcast(std::span<const std::byte> data, bool reliable) {
    // Nothing to send (e.g. a structural message with zero commands): skip, rather
    // than putting an empty packet on the wire to every connection.
    if (data.empty()) {
        return;
    }
    for (INetworkTransport::ConnectionId connection : m_connections) {
        m_transport->send(connection, data, reliable);
    }
}

std::weak_ptr<RigidBody> GameNetworkBase::findGridBody(std::uint64_t id) const {
    std::shared_ptr<Grid> grid{m_gameBase->getGridSubsystem()->getGridById(id).lock()};
    return grid ? grid->getRigidBody() : std::weak_ptr<RigidBody>{};
}

std::weak_ptr<RigidBody> GameNetworkBase::findCharacterBody(std::int32_t id) const {
    std::shared_ptr<Character> character{
        m_gameBase->m_characterSubsystem->getCharacterById(id).lock()};
    return character ? character->getRigidBody() : std::weak_ptr<RigidBody>{};
}

std::uint64_t GameNetworkBase::findGridIdForBody(const std::weak_ptr<RigidBody>& body) const {
    std::shared_ptr<Grid> grid{m_gameBase->getGridSubsystem()->getGridFromBody(body).lock()};
    return grid ? grid->uniqueId : DigibotInput::k_noLockTargetGridId;
}

std::shared_ptr<Digibot> GameNetworkBase::findDigibot(int characterId) const {
    std::shared_ptr<Character> character{
        m_gameBase->m_characterSubsystem->getCharacterById(characterId).lock()};
    return std::dynamic_pointer_cast<Digibot>(character);
}

DigibotController* GameNetworkBase::findDigibotController(int characterId) const {
    std::shared_ptr<Digibot> digibot{findDigibot(characterId)};
    return digibot ? digibot->getController() : nullptr;
}

CockpitDockingCoordinator::DockingStatus GameNetworkBase::captureDockingStatus(
    int characterId) const {
    std::shared_ptr<Digibot> digibot{findDigibot(characterId)};
    return digibot ? m_gameBase->m_cockpitDockingCoordinator->captureDockingStatus(digibot.get())
                   : CockpitDockingCoordinator::DockingStatus{};
}

void GameNetworkBase::forceDockingStatus(
    int characterId, const CockpitDockingCoordinator::DockingStatus& status) {
    std::shared_ptr<Digibot> digibot{findDigibot(characterId)};
    if (digibot) {
        m_gameBase->m_cockpitDockingCoordinator->forceDockingStatus(
            digibot, status, m_gameBase->getGridSubsystem());
    }
}

DigibotInput GameNetworkBase::captureResolvedInput(int characterId) const {
    DigibotInput input{};
    DigibotController* controller{findDigibotController(characterId)};
    if (controller) {
        input = controller->captureInput();
        input.m_lockTargetGridId = findGridIdForBody(controller->getTargetRigidBody());
    }
    return input;
}

void GameNetworkBase::applyResolvedInput(int characterId, const DigibotInput& input) {
    DigibotController* controller{findDigibotController(characterId)};
    if (!controller) {
        return;
    }
    std::weak_ptr<RigidBody> lockTarget{};
    if (input.m_lockTargetGridId != DigibotInput::k_noLockTargetGridId) {
        lockTarget = findGridBody(input.m_lockTargetGridId);
    }
    controller->applyInput(input, lockTarget);
}

std::vector<std::byte> GameNetworkBase::buildStructuralMessage(
    const std::vector<StructuralCommand>& commands) const {
    if (commands.empty()) {
        return {};
    }
    ByteWriter writer{};
    writer.write(static_cast<std::uint8_t>(MessageType::Structural));
    writer.write(static_cast<std::uint32_t>(commands.size()));
    for (const StructuralCommand& command : commands) {
        command.serialize(writer);
    }
    return writer.take();
}

void GameNetworkBase::receiveStructural(ByteReader& reader) {
    // Reliable, ordered: every command is kept and applied in arrival order.
    std::uint32_t count{0};
    if (!reader.read(count)) {
        return;
    }
    for (std::uint32_t ii = 0; ii < count; ii++) {
        StructuralCommand command{};
        if (!command.deserialize(reader)) {
            break;
        }
        m_receivedReliable.push_back(command);
    }
}

GameNetworkBase::ControlDesire GameNetworkBase::readControlDesire(Mode& mode) {
    bool wantsControl{mode.wantsCharacterControl()};
    int preferredCharacterId{-1};
    if (wantsControl && !mode.hasBoundCharacter()) {
        // Only need a target while still unbound: pick the nearest character.
        // Once bound, this stops being recomputed every frame.
        std::shared_ptr<Digibot> desired{mode.desiredCharacter().lock()};
        preferredCharacterId = desired ? desired->getUniqueId() : -1;
    }
    bool wasWanting{m_wasWantingCharacterControl};
    m_wasWantingCharacterControl = wantsControl;
    return {wantsControl, wasWanting, preferredCharacterId};
}

void GameNetworkBase::bindModeToLocalPlayer(Mode& mode, bool wantsControl) {
    if (!wantsControl) {
        mode.bindCharacter(std::weak_ptr<Digibot>{});
    } else if (!mode.hasBoundCharacter() && m_localPlayerId >= 0) {
        std::shared_ptr<Character> character{
            m_gameBase->m_characterSubsystem->getCharacterById(m_localPlayerId).lock()};
        mode.bindCharacter(std::dynamic_pointer_cast<Digibot>(character));
    }
    setLocalControlsCharacter(mode.isControllingCharacter());
}

void GameNetworkBase::framePoll() {
    m_transport->poll();

    for (const INetworkTransport::Event& event : m_transport->drainEvents()) {
        switch (event.m_type) {
        case INetworkTransport::EventType::Connected: {
            std::cout << "[net] connected: " << event.m_detail << std::endl;
            m_connections.push_back(event.m_connection);
            break;
        }
        case INetworkTransport::EventType::Disconnected: {
            std::cout << "[net] disconnected: " << event.m_detail << std::endl;
            std::erase(m_connections, event.m_connection);
            onDisconnected(event.m_connection);
            break;
        }
        }
    }

    for (INetworkTransport::Message& message : m_transport->drainMessages()) {
        ByteReader reader{message.m_data};
        std::uint8_t rawType{0};
        if (!reader.read(rawType)) {
            continue;
        }
        handleMessage(static_cast<MessageType>(rawType), reader, message);
    }
}

void GameNetworkBase::stepApply() {
    // The per-step contract: only inside a step-control window, and at most once
    // per step — the anchor advance below must stay one-to-one with the world's
    // integrations.
    assert(m_gameBase->isAtStepControlPoint());
    assert(m_gameBase->getPhysicsTick() != m_lastStepApplyTick);
    m_lastStepApplyTick = m_gameBase->getPhysicsTick();
    stepApplyRole();
}

void GameNetworkBase::frameSend() {
    std::uint64_t tick{m_gameBase->getPhysicsTick()};
    // Mid-step body state is torn (partially integrated), so wait for the next
    // frame; the message will carry the newer tick then.
    if (tick == m_lastSentTick || m_gameBase->isPhysicsStepInProgress()) {
        return;
    }
    m_lastSentTick = tick;
    if (m_connections.empty()) {
        return;
    }
    frameSendRole(tick);
}
