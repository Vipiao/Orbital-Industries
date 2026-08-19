// FrameProfiler.cpp
#include "FrameProfiler.h"

#include <cstdlib>
#include <iostream>

FrameProfiler::FrameProfiler() {
    const char* requested{std::getenv("OI_FRAME_PROFILE")};
    if (!requested || requested[0] == '\0') {
        return;
    }

    const std::filesystem::path outputPath{requested};
    m_file.open(outputPath);
    if (!m_file) {
        std::cerr << "[profile] could not write " << outputPath << std::endl;
        return;
    }

    m_enabled = true;
    glGenQueries(k_queryDepth, m_queries.data());

    const GLubyte* renderer{glGetString(GL_RENDERER)};
    m_file << "# renderer=" << (renderer ? reinterpret_cast<const char*>(renderer) : "unknown")
           << '\n';
    m_file << "frame,gpuMillis,submitMillis,frameMillis\n";

    std::cout << "[profile] journalling frames to " << outputPath << std::endl;
}

FrameProfiler::~FrameProfiler() {
    if (!m_enabled) {
        return;
    }

    // The loop is over, so anything still in flight has long since finished.
    for (int slot{0}; slot < k_queryDepth; ++slot) {
        collect(slot);
    }

    glDeleteQueries(k_queryDepth, m_queries.data());
    std::cout << "[profile] journalled " << m_frame << " frames" << std::endl;
}

void FrameProfiler::beginFrame() {
    if (!m_enabled) {
        return;
    }

    const std::chrono::time_point<std::chrono::steady_clock> opening{
        std::chrono::steady_clock::now()};
    // Nothing to measure against on the first frame; a zero marks it as such.
    m_frameMillis =
        m_frame == 0
            ? 0.0
            : std::chrono::duration<double, std::milli>(opening - m_frameOpened).count();
    m_frameOpened = opening;

    const int slot{static_cast<int>(m_frame % k_queryDepth)};
    collect(slot);

    m_outstanding[slot] = Outstanding{m_frame, 0.0, m_frameMillis, true};
    glBeginQuery(GL_TIME_ELAPSED, m_queries[slot]);
}

void FrameProfiler::endFrame() {
    if (!m_enabled) {
        return;
    }

    glEndQuery(GL_TIME_ELAPSED);

    const int slot{static_cast<int>(m_frame % k_queryDepth)};
    m_outstanding[slot].m_submitMillis =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                 m_frameOpened)
            .count();
    ++m_frame;
}

void FrameProfiler::collect(int slot) {
    if (!m_outstanding[slot].m_valid) {
        return;
    }

    // Asked for outright rather than polled for readiness: k_queryDepth frames
    // back is finished, and a sample dropped for being early would bias the run
    // towards its cheap frames, which is the opposite of what this measures.
    GLuint64 elapsedNanos{0};
    glGetQueryObjectui64v(m_queries[slot], GL_QUERY_RESULT, &elapsedNanos);

    m_file << m_outstanding[slot].m_frame << ','
           << static_cast<double>(elapsedNanos) * 1e-6 << ','
           << m_outstanding[slot].m_submitMillis << ','
           << m_outstanding[slot].m_frameMillis << '\n';
    // Flushed per frame so a run that dies in teardown keeps what it measured.
    m_file.flush();

    m_outstanding[slot].m_valid = false;
}
