// TimeHandler.h - Create this new file
#pragma once

#include <chrono>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <iostream>

class TimeHandler {
public:
    enum class Mode { NONE, RECORD, PLAY };

    TimeHandler(Mode mode = Mode::NONE, const std::filesystem::path& filepath = "recording_time/time_data.bin") 
        : m_mode(mode) {
        if (mode != Mode::NONE) {
            // Ensure the directory exists
            if (!std::filesystem::exists(filepath.parent_path())) {
                std::filesystem::create_directories(filepath.parent_path());
            }

            // In RECORD mode, delete the file if it exists
            if (mode == Mode::RECORD && std::filesystem::exists(filepath)) {
                std::filesystem::remove(filepath);
            }

            // Open the file in the appropriate mode
            m_file.open(filepath, (mode == Mode::RECORD ? std::ios::out : std::ios::in) | std::ios::binary);
            if (!m_file.is_open()) {
                throw std::runtime_error("Failed to open file: " + filepath.string());
            }
        }

        // Initialize the start time
        m_startTime = std::chrono::high_resolution_clock::now();
    }

    ~TimeHandler() {
        if (m_mode != Mode::NONE) {
            m_file.close();
        }
    }

    // Get current time point (high resolution clock)
    std::chrono::time_point<std::chrono::high_resolution_clock> now() {
        if (m_mode == Mode::RECORD) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            // Record the time as a duration since start (more reliable for playback)
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime - m_startTime);
            record(duration);
            return currentTime;
        } 
        else if (m_mode == Mode::PLAY) {
            // Playback the recorded duration and convert back to a time point
            auto duration = playback<std::chrono::nanoseconds>();
            return m_startTime + duration;
        } 
        else {
            return std::chrono::high_resolution_clock::now();
        }
    }

    // Calculate elapsed time since given time point
    template <typename DurationType = std::chrono::duration<double>>
    DurationType elapsed(const std::chrono::time_point<std::chrono::high_resolution_clock>& since) {
        return std::chrono::duration_cast<DurationType>(now() - since);
    }

private:
    template <typename T>
    void record(const T& data) {
        m_file.write(reinterpret_cast<const char*>(&data), sizeof(T));
    }

    template <typename T>
    T playback() {
        T data{};
        m_file.read(reinterpret_cast<char*>(&data), sizeof(T));
        if (m_file.eof() || !m_file.good()) {
            m_file.close();
            m_mode = Mode::NONE;
            // Fall back to real time if we run out of recorded data
            std::cout << "Warning: End of recorded time data reached, switching to real-time." << std::endl;
            return std::chrono::duration_cast<T>(
                std::chrono::high_resolution_clock::now() - m_startTime);
        }
        return data;
    }

    Mode m_mode;
    std::fstream m_file;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};