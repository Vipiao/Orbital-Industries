#include "StartupPrompt.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

std::string readLine() {
    std::string line{};
    std::getline(std::cin, line);
    return line;
}

// Asks until the user enters a valid port or accepts the default with an empty line.
std::uint16_t askPort(const std::string& label, std::uint16_t defaultPort) {
    while (true) {
        std::cout << label << " [" << defaultPort << "]: " << std::flush;
        std::string line{readLine()};
        if (line.empty()) {
            return defaultPort;
        }
        try {
            int port{std::stoi(line)};
            if (port >= 1 && port <= 65535) {
                return static_cast<std::uint16_t>(port);
            }
        } catch (const std::exception&) {
        }
        std::cout << "Invalid port; enter a number between 1 and 65535." << std::endl;
    }
}

std::string askIp(const std::string& defaultIp) {
    std::cout << "Server IP [" << defaultIp << "]: " << std::flush;
    std::string line{readLine()};
    return line.empty() ? defaultIp : line;
}

}  // namespace

namespace startupPrompt {

NetworkStartupConfig prompt() {
    NetworkStartupConfig config{};
    while (true) {
        std::cout << "Start as (s)erver or (c)lient? " << std::flush;
        std::string line{readLine()};
        if (line == "s" || line == "S" || line == "server") {
            config.m_role = NetworkStartupConfig::Role::Server;
            config.m_port = askPort("Port to listen on", config.m_port);
            return config;
        }
        if (line == "c" || line == "C" || line == "client") {
            config.m_role = NetworkStartupConfig::Role::Client;
            config.m_ip = askIp(config.m_ip);
            config.m_port = askPort("Server port", config.m_port);
            return config;
        }
        std::cout << "Please answer 's' or 'c'." << std::endl;
    }
}

}  // namespace startupPrompt
