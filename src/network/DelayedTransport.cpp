#include "DelayedTransport.h"

#include <utility>

DelayedTransport::DelayedTransport(std::unique_ptr<INetworkTransport> inner, double delayMs,
                                   double jitterMs, double dropRate, std::uint32_t seed)
    : m_inner{std::move(inner)},
      m_delayMs{delayMs},
      m_jitterMs{jitterMs},
      m_dropRate{dropRate},
      m_rng{seed} {}

void DelayedTransport::poll() {
    m_inner->poll();

    std::uniform_real_distribution<double> unit{0.0, 1.0};
    Clock::time_point now{Clock::now()};
    for (Message& message : m_inner->drainMessages()) {
        // Only the unreliable channel loses or reorders messages; the reliable
        // channel's guarantees (delivery, send order per connection) must survive
        // the simulation.
        if (!message.m_reliable && m_dropRate > 0.0 && unit(m_rng) < m_dropRate) {
            continue;
        }
        double jitter{m_jitterMs > 0.0 ? unit(m_rng) * m_jitterMs : 0.0};
        Clock::duration delay{std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>{m_delayMs + jitter})};
        Clock::time_point releaseAt{now + delay};
        if (message.m_reliable) {
            Clock::time_point& lastRelease{m_lastReliableRelease[message.m_connection]};
            releaseAt = std::max(releaseAt, lastRelease);
            lastRelease = releaseAt;
        }
        m_held.push_back(Held{releaseAt, std::move(message)});
    }
}

std::vector<INetworkTransport::Message> DelayedTransport::drainMessages() {
    Clock::time_point now{Clock::now()};
    std::vector<Message> ready{};
    std::vector<Held> stillHeld{};
    for (Held& held : m_held) {
        if (held.m_releaseAt <= now) {
            ready.push_back(std::move(held.m_message));
        } else {
            stillHeld.push_back(std::move(held));
        }
    }
    m_held = std::move(stillHeld);
    return ready;
}
