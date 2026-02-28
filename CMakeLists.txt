cmake_minimum_required(VERSION 3.16)

project(
  mujoco_evolve
  VERSION 0.1.0
  DESCRIPTION "Co-evolution of morphology and neural control using MuJoCo"
  LANGUAGES C CXX
)

# ── C++17 required for std::thread, std::mutex, std::optional, <random>, etc.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build MuJoCo from source (disable extras we don't need)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)

add_subdirectory(third_party/mujoco)

# ── Our library: collects all project source files
add_library(evo_lib STATIC
  src/sim/sim_utils.cpp
  src/morphology/body_gene.cpp
  src/morphology/mjcf_writer.cpp
  src/brain/neural_controller.cpp
  src/ga/genetic_algorithm.cpp
  src/utils/thread_pool.cpp
)

target_include_directories(evo_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(evo_lib PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(evo_lib PUBLIC mujoco)

# ── Main executable
add_executable(evo src/main.cpp)
target_link_libraries(evo PRIVATE evo_lib)
target_compile_options(evo PRIVATE -Wall -Wextra -Wpedantic)

# ── Testing (Google Test)
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
  GIT_SHALLOW    TRUE
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(evo_tests
  tests/test_mujoco.cpp
)
target_link_libraries(evo_tests PRIVATE evo_lib GTest::gtest_main)
target_compile_options(evo_tests PRIVATE -Wall -Wextra -Wpedantic)

include(GoogleTest)
gtest_discover_tests(evo_tests)
