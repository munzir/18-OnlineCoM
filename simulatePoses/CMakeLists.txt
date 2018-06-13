cmake_minimum_required(VERSION 2.8.12)

project(simulatePoses)

find_package(DART 6.5.0 REQUIRED COMPONENTS utils-urdf gui)

add_compile_options(-std=c++11 -DCMAKE_BUILD_TYPE=Release)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
