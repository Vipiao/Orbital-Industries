// FrameProfiler.h
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>

#include <glad/glad.h>

/**
 * @brief Per-frame GPU and CPU cost, journalled to CSV.
 *
 * Carries its own clock rather than reading the engine's. A replay drives
 * TimeHandler from a recording, so everything derived from it -- the measured
 * frame rate, the frame start, the paced clock -- reports the run that was
 * recorded rather than the run happening now, and the GPU never appears in it
 * at all. Measuring what a change costs needs a clock the recording does not
 * reach.
 *
 * The GPU side is one GL_TIME_ELAPSED query per frame, read back several frames
 * later so waiting on a result never stalls the pipeline. It holds the wall time
 * the GPU spends between the two ends of the query, idle included, so it wants
 * vsync off to mean anything: with the swap throttled, the query closes when the
 * display allows rather than when the work is done.
 *
 * Written and flushed a frame at a time rather than gathered and dumped at the
 * end, so a run that dies in teardown still leaves everything it measured.
 *
 * Switched on by naming an output file in OI_FRAME_PROFILE, so one build can be
 * measured before and after a change without being rebuilt to toggle it. Unset,
 * nothing is opened and the frame carries no queries.
 */
class FrameProfiler {
public:
    // Reads OI_FRAME_PROFILE. Empty or unset leaves the profiler inert. Needs a
    // current GL context: it allocates its queries here.
    FrameProfiler();
    ~FrameProfiler();

    FrameProfiler(const FrameProfiler&) = delete;
    FrameProfiler& operator=(const FrameProfiler&) = delete;

    bool isEnabled() const { return m_enabled; }

    // Bracket the GPU work of one frame. Must not span the buffer swap, or the
    // query holds the presentation wait rather than the drawing.
    void beginFrame();
    void endFrame();

private:
    // Frames a query is left alone before its result is read. Enough that the
    // GPU is well past it, so the read returns rather than blocking on work
    // still in flight.
    static constexpr int k_queryDepth{4};

    // What a slot knows before its GPU result comes back.
    struct Outstanding {
        uint64_t m_frame{};
        double m_submitMillis{};
        double m_frameMillis{};
        bool m_valid{false};
    };

    // Writes out the query in this slot if one is outstanding.
    void collect(int slot);

    bool m_enabled{false};
    std::ofstream m_file{};

    std::array<GLuint, k_queryDepth> m_queries{};
    std::array<Outstanding, k_queryDepth> m_outstanding{};

    uint64_t m_frame{0};
    // Opening of the frame being measured. Its gap to the next opening is the
    // whole frame, which the bracket below does not span.
    std::chrono::time_point<std::chrono::steady_clock> m_frameOpened{};
    double m_frameMillis{0.0};
};
