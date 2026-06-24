cmake_minimum_required(VERSION 3.16)
project(OrbitalIndustries)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific flags
if(MSVC)
    add_compile_options(/W4 /O2)
    set(CMAKE_CXX_FLAGS_DEBUG "/Zi /Od /MDd")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /MD /DNDEBUG")
else()
    # GCC/Clang flags
    add_compile_options(-Wall -Wextra)
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unknown-pragmas)
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O2 -DNDEBUG")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Engine location — set in CMakeUserPresets.json (gitignored), Option 3.
set(ENGINE_DIR "" CACHE PATH "Path to the graphics engine repo")
add_subdirectory(${ENGINE_DIR} ${CMAKE_BINARY_DIR}/engine-build)

# Specify output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Game's own includes. glad/glm and all libs come from the engine target.
include_directories(${CMAKE_SOURCE_DIR})
include_directories("${CMAKE_SOURCE_DIR}/include")
if(WIN32)
    include_directories("C:/msys64/ucrt64/include")
    link_directories("C:/msys64/ucrt64/lib")
endif()

# Manually list source files
file(GLOB_RECURSE SRC_FILES
    "${CMAKE_SOURCE_DIR}/main.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Add the executable
add_executable(OrbitalIndustries ${SRC_FILES})

# Everything comes through the engine target's PUBLIC properties.
target_link_libraries(OrbitalIndustries PRIVATE GraphicsEngine::graphics)
