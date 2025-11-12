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

# Specify output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Common include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("${CMAKE_SOURCE_DIR}/external/glad/include")
include_directories("${CMAKE_SOURCE_DIR}/external/glm")

# Platform-specific configuration
if(WIN32)
    message(STATUS "Configuring for Windows")
    include_directories("C:/msys64/ucrt64/include")
    link_directories("C:/msys64/ucrt64/lib")
elseif(UNIX)
    message(STATUS "Configuring for Linux")
    # Find packages using CMake's package finder
    find_package(assimp REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(OpenGL REQUIRED)
    include_directories(${ASSIMP_INCLUDE_DIRS})
endif()

# Manually list source files
file(GLOB_RECURSE SRC_FILES
    "${CMAKE_SOURCE_DIR}/main.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/external/glad/src/glad.c"
)

# Add the executable
add_executable(OrbitalIndustries ${SRC_FILES})

# Platform-specific linking
if(WIN32)
    target_link_libraries(OrbitalIndustries
        "${CMAKE_SOURCE_DIR}/lib/libglfw3.a"
        -lassimp
        -lopengl32
        -lgdi32
        -luser32
        -lshell32
    )
elseif(UNIX)
    target_link_libraries(OrbitalIndustries
        glfw
        ${ASSIMP_LIBRARIES}
        ${OPENGL_LIBRARIES}
    )
endif()
