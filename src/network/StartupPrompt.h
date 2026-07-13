// StartupPrompt.h — blocking stdin dialog asking for server/client role and address.
#pragma once

#include "NetworkStartupConfig.h"

namespace startupPrompt {

// Runs before any engine/window init; re-asks until the input parses.
NetworkStartupConfig prompt();

}  // namespace startupPrompt
