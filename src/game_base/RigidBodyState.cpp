#include "RigidBodyState.h"

#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"

#include <cmath>

RigidBodyState RigidBodyState::capture(const RigidBody& body) {
    return RigidBodyState{body.getPosition(), body.getOrientation(), body.m_velocity,
                          body.getAngularMomentumBody()};
}

void RigidBodyState::apply(const std::weak_ptr<RigidBody>& bodyWeak,
                           PhysicsEngine& physicsEngine) const {
    std::shared_ptr<RigidBody> body{bodyWeak.lock()};
    if (!body) {
        return;
    }
    body->setPosition(m_position);
    body->m_velocity = m_velocity;
    body->setOrientation(m_orientation);
    body->setAngularMomentumBody(m_angularMomentumBody);
    physicsEngine.updateColliderTransform(bodyWeak);
}

void RigidBodyState::serialize(ByteWriter& writer) const {
    writer.write(m_position.x);
    writer.write(m_position.y);
    writer.write(m_position.z);
    writer.write(m_orientation.w);
    writer.write(m_orientation.x);
    writer.write(m_orientation.y);
    writer.write(m_orientation.z);
    writer.write(m_velocity.x);
    writer.write(m_velocity.y);
    writer.write(m_velocity.z);
    writer.write(m_angularMomentumBody.x);
    writer.write(m_angularMomentumBody.y);
    writer.write(m_angularMomentumBody.z);
}

bool RigidBodyState::deserialize(ByteReader& reader) {
    return reader.read(m_position.x) && reader.read(m_position.y) &&
           reader.read(m_position.z) && reader.read(m_orientation.w) &&
           reader.read(m_orientation.x) && reader.read(m_orientation.y) &&
           reader.read(m_orientation.z) && reader.read(m_velocity.x) &&
           reader.read(m_velocity.y) && reader.read(m_velocity.z) &&
           reader.read(m_angularMomentumBody.x) &&
           reader.read(m_angularMomentumBody.y) &&
           reader.read(m_angularMomentumBody.z);
}

bool RigidBodyState::isFinite() const {
    return std::isfinite(m_position.x) && std::isfinite(m_position.y) &&
           std::isfinite(m_position.z) && std::isfinite(m_orientation.w) &&
           std::isfinite(m_orientation.x) && std::isfinite(m_orientation.y) &&
           std::isfinite(m_orientation.z) && std::isfinite(m_velocity.x) &&
           std::isfinite(m_velocity.y) && std::isfinite(m_velocity.z) &&
           std::isfinite(m_angularMomentumBody.x) &&
           std::isfinite(m_angularMomentumBody.y) &&
           std::isfinite(m_angularMomentumBody.z);
}
