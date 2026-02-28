MuJoCo Fundamentals
===================

This page explains how MuJoCo is used within MuJoCo Evolve: how models are
loaded, how the simulation loop works, and the key data structures involved.
It also serves as a reference for the patterns used throughout the codebase.

.. contents:: On this page
   :local:
   :depth: 2

Core Concepts
-------------

MuJoCo separates **model description** from **simulation state**:

``mjModel``
   A read-only struct compiled from MJCF XML (or built programmatically via
   ``mjSpec``). Contains all physics parameters: masses, geometries, joint axes,
   actuator gains, sensor definitions, and solver settings. Created once per
   morphology; can be shared across threads.

``mjData``
   A mutable struct holding the live simulation state: joint positions
   (``qpos``), velocities (``qvel``), actuator commands (``ctrl``), sensor
   readings (``sensordata``), contact forces, and all intermediate computation
   buffers. Each thread must have its own ``mjData`` instance.

Loading a Model from Memory
---------------------------

MuJoCo normally loads MJCF XML from disk, but a **Virtual File System (VFS)**
allows loading from an in-memory string. This is the pattern used throughout the
project, since the genetic algorithm generates MJCF dynamically:

.. code-block:: cpp

   mjVFS vfs;
   mj_defaultVFS(&vfs);
   mj_addBufferVFS(&vfs, "model.xml", xml_string, xml_length);

   char error[1000] = "";
   mjModel* m = mj_loadXML("model.xml", &vfs, error, sizeof(error));

   if (!m) {
       // Handle error — error[] contains the diagnostic message
   }

``mj_loadXML`` parses the XML, validates it, computes derived quantities (inertia
tensors, tendon paths, etc.), and returns a compiled ``mjModel``. If parsing or
validation fails, it returns ``nullptr`` and writes a human-readable diagnostic
into the error buffer.

The Simulation Loop
-------------------

Once a model is compiled, simulation follows a straightforward loop:

.. code-block:: cpp

   mjData* d = mj_makeData(m);    // allocate state for this model

   mj_forward(m, d);              // populate initial sensors and kinematics

   while (d->time < duration) {
       // Read sensors
       // ... = d->sensordata[0..m->nsensordata-1]

       // Set actuator commands
       // d->ctrl[0..m->nu-1] = ...

       mj_step(m, d);             // advance one timestep
   }

Key points:

- Call ``mj_forward`` **once before the loop** to populate ``sensordata`` and
  ``xpos`` with valid initial values. Without this, the first control decision
  reads zeros.
- Each ``mj_step`` call advances by ``m->opt.timestep`` seconds (default 0.002 s).
- Use ``m->nsensordata`` (total scalar readings), **not** ``m->nsensor`` (number
  of sensor objects). A single ``framequat`` sensor produces 4 scalars.

Resource Management
-------------------

Every allocation must be paired with the corresponding free call:

.. code-block:: cpp

   mj_deleteData(d);      // free simulation state
   mj_deleteModel(m);     // free compiled model
   mj_deleteVFS(&vfs);    // free virtual file system

Missing any of these causes memory leaks. The project uses RAII wrappers (or
explicit cleanup in ``evaluate()``) to ensure resources are released even when
errors occur.

Determinism
-----------

MuJoCo's physics engine is fully deterministic: two runs from the same initial
state with the same model always produce **bit-identical** results. This property
is not a design requirement for the evolutionary algorithm (which is stochastic by
nature), but it is essential for writing reliable unit tests.

The ``Determinism`` test in ``tests/test_mujoco.cpp`` verifies this by running two
independent 1000-step simulations and asserting exact floating-point equality on
the final joint positions.

Thread Safety
-------------

Simulation functions (``mj_step``, ``mj_forward``, etc.) are thread-safe **when
each thread uses its own** ``mjData``. A single ``mjModel`` can be shared
read-only across threads.

However, **model compilation is not thread-safe**: ``mj_loadXML`` and
``mj_compile`` must either run in the main thread or be protected by a mutex.
See the multithreading design in the planning document for details.

Global callbacks (``mjcb_control``, ``mjcb_passive``, etc.) are process-wide
function pointers and cannot vary per thread. This project avoids callbacks
entirely, using an explicit sensor → neural network → control loop instead.

Test Suite Reference
--------------------

The file ``tests/test_mujoco.cpp`` contains the foundational MuJoCo tests:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Test Name
     - What it verifies
   * - ``MuJoCo.VersionString``
     - ``mj_versionString()`` returns ``"3.2.7"``
   * - ``MuJoCo.VersionNumber``
     - ``mj_version()`` returns a positive integer
   * - ``MuJoCo.LoadModelFromString``
     - A minimal MJCF model loads via VFS; ``njnt``, ``nu``, ``nsensordata``
       match expectations
   * - ``MuJoCo.StepSimulation``
     - A capsule falls under gravity over 500 timesteps
   * - ``MuJoCo.Determinism``
     - Two identical 1000-step runs produce bit-identical ``qpos``

All tests use a shared minimal model: a capsule on a ground plane with one slide
joint, one motor, and two sensors (``jointpos`` + ``jointvel``).
