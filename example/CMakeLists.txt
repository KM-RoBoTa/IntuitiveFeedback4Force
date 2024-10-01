cmake_minimum_required(VERSION 3.20)
project(sensors)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Create list of additional sources in a different folder than the main.cpp
set(additional_sources
    force_sensors.cpp
)
list(TRANSFORM additional_sources PREPEND "../readForces_cppCode/")

# Source files
add_executable(${PROJECT_NAME}
            main.cpp
            ${additional_sources}
)

# Add header files
include_directories(
    ${PROJECT_SOURCE_DIR}/../readForces_cppCode
)
