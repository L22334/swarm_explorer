cmake_minimum_required(VERSION 3.14)
project("demo")

include("colcon.cmake")

colcon_add_subdirectories(
        BUILD_BASE "${PROJECT_SOURCE_DIR}/build"
        BASE_PATHS "${PROJECT_SOURCE_DIR}/src/"
        # --packages-select
)