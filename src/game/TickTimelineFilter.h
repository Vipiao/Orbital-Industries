// TickTimelineFilter.h — keeps a client's tick phase aligned with the server's
// (notes/multiplayer_implementation.md §6). Peers' tick clocks are never phase
// locked, so the accumulated offset drifts. Rather than shifting object positions
// to hide that drift (which is only common-mode when every object is refreshed the
// same tick), this reports a small nudge to *when the next physics step runs*, so
// the client runs microscopically fast or slow until the drift is zero. Correcting
// in time keeps the whole world on one integer tick timeline. Pure arithmetic: no
// transport, no bodies, no clock.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

class TickTimelineFilter {
public:
    // How to align the local tick phase after one received snapshot.
    struct Alignment {
        double m_scheduleNudgeTicks{0.0};  // shift the next step by this many ticks of phase
        double m_driftTicks{0.0};          // accumulated drift from baseline (for logging)
        bool m_resync{false};              // gross desync: baseline re-based in one step
    };

    // Feed the tick pair of one received snapshot, in arrival order.
    //   drift = baseline − (serverTick − clientTick),  baseline fixed at first call
    //   nudge = clamp(gain · drift)                    (positive drift = client ahead)
    // A drift beyond s_resyncTicks is a stall, not phase wander: the baseline is
    // re-based in one step and m_resync is set so the caller re-anchors what it
    // predicts. Otherwise the proportional nudge (slow gain + clamp) bleeds the
    // drift off gradually — the low-pass that a spatial EMA used to provide.
    Alignment update(std::uint64_t serverTick, std::uint64_t clientTick) {
        double server{static_cast<double>(serverTick)};
        double client{static_cast<double>(clientTick)};
        if (!m_hasBaseline) {
            m_baselineTickDiff = server - client;
            m_hasBaseline = true;
        }
        double drift{m_baselineTickDiff - (server - client)};
        Alignment result{};
        result.m_driftTicks = drift;
        if (std::abs(drift) > s_resyncTicks) {
            m_baselineTickDiff = server - client;
            result.m_resync = true;
            return result;
        }
        result.m_scheduleNudgeTicks =
            std::clamp(s_gain * drift, -s_maxNudgeTicks, s_maxNudgeTicks);
        return result;
    }

private:
    // Fraction of the observed drift corrected per snapshot, the largest single
    // nudge allowed (so a jittery arrival cannot yank the schedule), and the drift
    // past which the gradual nudge gives way to a one-step re-base.
    static constexpr double s_gain{0.1};
    static constexpr double s_maxNudgeTicks{0.25};
    static constexpr double s_resyncTicks{5.0};

    bool m_hasBaseline{false};
    double m_baselineTickDiff{0.0};  // first server tick − first client tick
};
