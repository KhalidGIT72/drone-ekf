cmake_minimum_required(VERSION 3.10)
project(drone_ekf)

set(CMAKE_CXX_STANDARD 17)

# Path to Eigen
include_directories("C:/libs/eigen-5.0.0")

# Add executable
add_executable(drone_ekf src/main.cpp)