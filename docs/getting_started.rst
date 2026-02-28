Getting Started
===============

This guide walks through setting up the MuJoCo Evolve development environment
from a fresh clone to a passing test suite.

.. contents:: On this page
   :local:
   :depth: 2

Prerequisites
-------------

============  ==============================================
Requirement   Version / Details
============  ==============================================
OS            Linux (tested on Ubuntu 22.04, x86_64)
Compiler      GCC 11+ with C++17 support
CMake         3.22+
Python        3.10+ (documentation toolchain only)
Doxygen       1.9+ (API reference generation)
============  ==============================================

Project Layout
--------------

.. code-block:: text

   mujoco_evolve/
   ├── CMakeLists.txt          # Top-level build configuration
   ├── Doxyfile                # Doxygen config for C++ API docs
   ├── src/
   │   ├── main.cpp
   │   ├── ga/                 # Genetic algorithm
   │   ├── morphology/         # Genotype encoding & MJCF generation
   │   ├── brain/              # Neural network controller
   │   ├── sim/                # MuJoCo wrappers & evaluation
   │   └── utils/              # Threading, RNG, logging
   ├── include/                # Public headers (mirrors src/)
   ├── tests/                  # GoogleTest suite
   ├── docs/                   # Sphinx documentation (this site)
   ├── third_party/mujoco/     # MuJoCo 3.2.7 source (git-ignored)
   └── agent_notes/            # Research & planning notes

Clone and Build
---------------

**1. Clone the repository** (if you haven't already):

.. code-block:: bash

   git clone <repo-url> mujoco_evolve
   cd mujoco_evolve

**2. Fetch MuJoCo source** into ``third_party/``:

.. code-block:: bash

   git clone --depth 1 --branch 3.2.7 \
       https://github.com/google-deepmind/mujoco.git third_party/mujoco

**3. Configure and build:**

.. code-block:: bash

   cmake -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j$(nproc)

**4. Verify the build:**

.. code-block:: bash

   ./build/evo
   # mujoco_evolve v0.1.0
   # MuJoCo version: 3.2.7

Running Tests
-------------

The project uses GoogleTest (fetched automatically via CMake ``FetchContent``).

.. code-block:: bash

   cd build && ctest --output-on-failure

All tests in ``tests/test_mujoco.cpp`` should pass. These cover model loading,
physics stepping, and simulation determinism — see :doc:`mujoco_fundamentals`
for details on what each test verifies.

Build System Notes
------------------

- **C++17** is required for ``std::thread``, ``std::mutex``, ``std::optional``,
  structured bindings, and ``<random>``.
- MuJoCo 3.2.7 is built from source via ``add_subdirectory(third_party/mujoco)``
  and linked through the ``mujoco`` CMake target.
- Compiler warnings (``-Wall -Wextra -Wpedantic``) are applied only to project
  targets via ``target_compile_options(... PRIVATE ...)``. This prevents MuJoCo's
  own C sources from triggering ``-Werror=unused-parameter`` under our strict flags.
- GoogleTest v1.15.2 is fetched via ``FetchContent`` — no manual install needed.
- Release builds use ``-O3`` optimisation.

Memory Checking
---------------

Two tools are available for catching memory errors:

.. list-table::
   :header-rows: 1
   :widths: 15 30 30 25

   * - Tool
     - What it does
     - How to use
     - Trade-offs
   * - **AddressSanitizer**
     - Compiler-instrumented checker. Detects leaks, use-after-free, buffer
       overflows, and stack overflows.
     - Add ``-fsanitize=address`` to compile **and** link flags. Run the binary
       normally — errors are printed to stderr at exit.
     - ~2× slower, ~3× memory. Built into GCC/Clang. **Recommended default.**
   * - **Valgrind**
     - External runtime tool. Detects leaks, uninitialized reads, and invalid
       accesses.
     - ``sudo apt install valgrind`` then
       ``valgrind --leak-check=full ./build/evo_tests``
     - ~20× slower. More thorough for uninitialized memory.

To enable ASan for a debug build:

.. code-block:: bash

   cmake -B build-debug \
       -DCMAKE_BUILD_TYPE=Debug \
       -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"
   cmake --build build-debug -j$(nproc)
   cd build-debug && ctest --output-on-failure

Use Valgrind only for deep investigations or pre-release checks.
