Getting Started
===============

This page covers project setup, building from source, and verifying that MuJoCo works correctly.

.. contents:: On this page
   :local:
   :depth: 2

Prerequisites
-------------

- **OS:** Linux (tested on Ubuntu 22.04, x86_64)
- **Compiler:** GCC 11+ (C++17 required)
- **CMake:** 3.22+
- **Python 3.10+** (for documentation only)

Project Structure
-----------------

.. code-block:: text

   mujoco_evolve/
   ├── CMakeLists.txt          # Top-level build configuration
   ├── Doxyfile                # Doxygen config (XML output for Breathe)
   ├── src/
   │   ├── main.cpp
   │   ├── ga/                 # Genetic algorithm logic
   │   ├── morphology/         # Genotype graph, MJCF generation
   │   ├── brain/              # Neural network implementation
   │   ├── sim/                # MuJoCo wrappers, evaluation
   │   └── utils/              # Threading, RNG, logging
   ├── include/                # Public headers mirroring src/
   ├── tests/                  # GoogleTest suite
   ├── docs/                   # Sphinx documentation (this site)
   ├── third_party/mujoco/     # MuJoCo 3.2.7 source (git-ignored)
   └── agent_notes/            # Research & planning docs

Building
--------

**1. Clone MuJoCo into third_party/**

.. code-block:: bash

   git clone --depth 1 --branch 3.2.7 \
       https://github.com/google-deepmind/mujoco.git third_party/mujoco

**2. Configure and build**

.. code-block:: bash

   cmake -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j$(nproc)

**3. Run the executable**

.. code-block:: bash

   ./build/evo
   # Expected output:
   # mujoco_evolve v0.1.0
   # MuJoCo version: 3.2.7

**4. Run the test suite**

.. code-block:: bash

   cd build && ctest --output-on-failure

CMake Configuration
^^^^^^^^^^^^^^^^^^^

Key points about the build system:

- **C++17** is required (``CMAKE_CXX_STANDARD 17``).
- MuJoCo 3.2.7 is built from source via ``add_subdirectory(third_party/mujoco)``
  and linked through the ``mujoco`` CMake target.
- Compiler warnings (``-Wall -Wextra -Wpedantic``) are applied only to our own
  targets via ``target_compile_options(... PRIVATE ...)`` to avoid triggering
  warnings inside MuJoCo's C sources.
- GoogleTest v1.15.2 is fetched via ``FetchContent``.

MuJoCo Sanity Check
--------------------

The GoogleTest suite in ``tests/test_mujoco.cpp`` verifies that MuJoCo loads,
steps, and behaves deterministically. Below is a walkthrough.

Minimal MJCF Model
^^^^^^^^^^^^^^^^^^^

All tests share a minimal model defined as a compile-time string constant:

.. code-block:: cpp

   static constexpr char kMinimalXml[] = R"(
   <mujoco>
     <worldbody>
       <light pos="0 0 3" dir="0 0 -1"/>
       <geom type="plane" size="5 5 0.1"/>
       <body name="capsule" pos="0 0 1">
         <joint name="slide" type="slide" axis="0 0 1"/>
         <geom type="capsule" size="0.1" fromto="0 0 0 0 0 0.5" mass="1"/>
       </body>
     </worldbody>
     <actuator>
       <motor joint="slide" gear="100"/>
     </actuator>
     <sensor>
       <jointpos joint="slide"/>
       <jointvel joint="slide"/>
     </sensor>
   </mujoco>
   )";

This defines: a capsule on a ground plane, one slide joint (vertical), one motor,
and two sensors (position + velocity). It's the simplest possible physics scenario.

Loading from Memory (``LoadModelFromString``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MuJoCo normally loads XML from disk, but a **Virtual File System (VFS)**
lets us load from an in-memory string — essential for our GA where MJCF is
generated dynamically:

.. code-block:: cpp

   mjVFS vfs;
   mj_defaultVFS(&vfs);
   mj_addBufferVFS(&vfs, "test.xml", kMinimalXml, strlen(kMinimalXml));

   char error[1000] = "";
   mjModel* m = mj_loadXML("test.xml", &vfs, error, sizeof(error));
   // m->njnt == 1, m->nu == 1, m->nsensordata == 2

``mj_loadXML`` parses the XML, validates it, and produces an ``mjModel`` — a
**read-only** struct with all physics parameters (masses, geometries, joint axes, etc.).
If parsing fails, ``m`` is ``nullptr`` and ``error`` describes the problem.

Stepping Physics (``StepSimulation``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: cpp

   mjData* d = mj_makeData(m);   // allocate mutable simulation state
   double z0 = d->qpos[0];       // initial joint position

   for (int i = 0; i < 500; ++i)
       mj_step(m, d);             // advance one timestep (default 0.002 s)

   double z1 = d->qpos[0];
   // z1 < z0 — the capsule fell under gravity

``mjData`` holds mutable state: joint positions (``qpos``), velocities (``qvel``),
sensor readings (``sensordata``), and actuator commands (``ctrl``). Each ``mj_step``
call advances by one timestep. With no motor input, gravity pulls the capsule down.

Determinism (``Determinism``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MuJoCo's physics are fully deterministic — two runs from the same initial state
always produce **bit-identical** results. This is not a design requirement for our
GA (evolution is stochastic), but it's critical for reliable unit tests:

.. code-block:: cpp

   double run1 = run_sim();   // 1000 steps, return final qpos[0]
   double run2 = run_sim();   // same again
   EXPECT_DOUBLE_EQ(run1, run2);

Cleanup
^^^^^^^

Every allocation must be paired with its corresponding free:

.. code-block:: cpp

   mj_deleteData(d);     // free simulation state
   mj_deleteModel(m);    // free compiled model
   mj_deleteVFS(&vfs);   // free virtual file system

Memory Checking
^^^^^^^^^^^^^^^

Two tools for catching leaks:

.. list-table::
   :header-rows: 1
   :widths: 15 30 30 25

   * - Tool
     - What it does
     - How to use
     - Trade-offs
   * - **AddressSanitizer**
     - Compiler-instrumented checker. Detects leaks, use-after-free, buffer overflows.
     - Add ``-fsanitize=address`` to compile and link flags. Run your binary normally.
     - ~2× slower, ~3× more memory. Built into GCC/Clang. **Recommended as default.**
   * - **Valgrind**
     - External runtime tool. Detects leaks, uninitialized reads, invalid accesses.
     - ``sudo apt install valgrind`` then ``valgrind --leak-check=full ./build/evo_tests``
     - ~20× slower. More thorough for uninitialized memory.

**Recommendation:** Enable ASan in a CMake debug preset
(``-DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"``) and run tests
through it routinely. Use Valgrind for deep investigations or pre-release checks.
