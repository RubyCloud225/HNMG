cmake_minimum_required(VERSION 3.16)
project(HNMG VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(SFML 2 REQUIRED COMPONENTS graphics window system)

# Source files
add_executable(HNMG
    c_space.cpp
)

# Include directories
target_include_directories(HNMG PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link libraries
target_link_libraries(HNMG PRIVATE
    Eigen3::Eigen
    sfml-graphics
    sfml-window
    sfml-system
)

# Compiler flags for performance on ARM
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_compile_options(HNMG PRIVATE -O3 -march=native)
else()
    target_compile_options(HNMG PRIVATE -O3)
endif()
