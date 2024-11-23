cmake_minimum_required(VERSION 3.15)
project(StaticEsdfSim)

set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME}
    example.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)