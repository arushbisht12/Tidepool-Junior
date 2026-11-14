cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tidepool_junior)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include(FetchContent)
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Eigen)


target_sources(app PRIVATE 
    src/main.c
    src/ekf_bridge.cpp
)


target_link_libraries(app PRIVATE
    Eigen3::Eigen
)