# CMakeList.txt : CMake project for twojugsproblem, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project ("twojugsproblem")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source to this project's executable.
add_executable (twojugsproblem "twojugsproblem.cpp" "jug_state_space.h" "jug_state_space.cpp"  "solver.h")

# TODO: Add tests and install targets if needed.
