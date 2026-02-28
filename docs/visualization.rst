Visualization
=============

MuJoCo provides both an interactive viewer (``simulate``) and a programmatic
rendering API. This project uses both: the viewer for manual inspection and
a headless rendering pipeline for automated testing.

.. contents:: On this page
   :local:
   :depth: 2

Interactive Viewer (simulate)
-----------------------------

MuJoCo ships a built-in viewer application called **simulate**. It is compiled
as part of our build when ``MUJOCO_BUILD_SIMULATE`` is ``ON`` in CMakeLists.txt
(the default). GLFW is required.

**Install GLFW** (Ubuntu):

.. code-block:: bash

   sudo apt install libglfw3-dev

**Launch the viewer** with any MJCF model:

.. code-block:: bash

   ./build/bin/simulate models/test_capsule.xml

The viewer provides:

- Real-time 3D rendering of the scene.
- Play/pause physics with **Space**.
- Actuator control sliders in the right-hand panel.
- Camera manipulation via mouse drag (rotate, pan, zoom).
- Contact force and constraint visualization toggles.

.. note::

   The simulate app requires a display server (X11 or Wayland). Under WSL2,
   it depends on WSLg and working GPU pass-through — it may segfault (exit
   code 139) if the OpenGL driver is not available. The headless tests below
   are the reliable alternative.

Test Model
----------

A reusable test scene lives at ``models/test_capsule.xml``:

.. code-block:: xml

   <mujoco model="test_capsule">
     <option timestep="0.002"/>
     <worldbody>
       <light pos="0 0 3" dir="0 0 -1"/>
       <geom type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
       <body name="capsule" pos="0 0 1">
         <joint name="slide" type="slide" axis="0 0 1"/>
         <geom type="capsule" size="0.1" fromto="0 0 0 0 0 0.5" mass="1"
               rgba="0.2 0.6 0.9 1"/>
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

This is the same capsule-on-plane scene used in the physics tests (see
:doc:`mujoco_fundamentals`), saved as a standalone file.

Headless Rendering Tests
------------------------

For automated testing (including CI where no display may be available), the
project uses **offscreen rendering** via a hidden GLFW window. This creates a
valid OpenGL context without showing anything on screen.

The tests live in ``tests/test_rendering.cpp`` and cover:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Test
     - What it verifies
   * - ``OffscreenBufferExists``
     - The offscreen framebuffer has non-zero dimensions after
       ``mjr_makeContext``.
   * - ``RenderProducesNonBlackImage``
     - Rendering the scene produces pixel data that is not all-black — i.e.
       the light, ground plane, and geometry were actually drawn.
   * - ``RenderAfterPhysicsStep``
     - Rendering still works correctly after stepping physics for 100 frames,
       catching state-management bugs in the visualisation pipeline.

**How the hidden-window approach works:**

.. code-block:: cpp

   // Create an invisible window just for the GL context.
   glfwInit();
   glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
   glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
   GLFWwindow* window = glfwCreateWindow(800, 600, "offscreen", NULL, NULL);
   glfwMakeContextCurrent(window);

   // Now MuJoCo's mjr_* rendering functions work normally.
   mjr_setBuffer(mjFB_OFFSCREEN, &con);
   mjr_render(viewport, &scn, &con);

   // Read back pixels.
   mjr_readPixels(rgb, nullptr, viewport, &con);

The test binary links against ``glfw`` (already built as part of the simulate
target) in addition to ``mujoco`` and ``GTest``.

Build Dependencies
------------------

The rendering pipeline adds one system dependency beyond the base project:

============  ==============================================
Package       Purpose
============  ==============================================
libglfw3-dev  OpenGL window/context creation (used by both
              the simulate viewer and headless tests)
============  ==============================================

On CI, GLFW is installed alongside a virtual framebuffer (``xvfb``) so that
the hidden-window tests can create an OpenGL context without a real display:

.. code-block:: bash

   sudo apt-get install -y libglfw3-dev xvfb
   xvfb-run ./build/evo_tests
