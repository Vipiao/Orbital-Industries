cmake_minimum_required(VERSION 3.16)
project(OrbitalIndustries)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify output directory for the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Include directories for header files
include_directories(${CMAKE_SOURCE_DIR})
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("C:/Users/user/OneDrive/cpp/libraries/glad/include")
include_directories("C:/Users/user/OneDrive/cpp/libraries/glm")
include_directories("C:/msys64/ucrt64/include")

# Add UCRT64 library path
link_directories("C:/msys64/ucrt64/lib")

# Manually list source files
file(GLOB_RECURSE SRC_FILES
    "${CMAKE_SOURCE_DIR}/main.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "C:/Users/user/OneDrive/cpp/libraries/glad/src/glad.c"
)

# Add the GLAD source file

# Add the executable
add_executable(OrbitalIndustries ${SRC_FILES})

# Link with system libraries
target_link_libraries(OrbitalIndustries
    "${CMAKE_SOURCE_DIR}/lib/libglfw3.a"
    -lassimp
    -lopengl32
    -lgdi32
    -luser32
    -lshell32
)