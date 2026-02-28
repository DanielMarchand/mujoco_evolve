Documentation Infrastructure
============================

This page describes how the project documentation is built, how to add new
pages, and how the C++ API reference is generated.

.. contents:: On this page
   :local:
   :depth: 2

Stack Overview
--------------

The documentation uses the same toolchain as MuJoCo itself:

============  ==============================================================
Component     Role
============  ==============================================================
Sphinx        Documentation generator — converts ``.rst`` files to HTML
Furo          Clean, responsive HTML theme
Breathe       Bridge between Doxygen XML output and Sphinx directives
Doxygen       Extracts API documentation from C++ source comments
KaTeX         Client-side math rendering (via ``sphinxcontrib.katex``)
Copy button   Adds a "copy" button to code blocks (``sphinx_copybutton``)
============  ==============================================================

Directory Layout
----------------

.. code-block:: text

   docs/
   ├── conf.py             # Sphinx configuration
   ├── Makefile            # Build targets (html, clean, doxygen)
   ├── index.rst           # Landing page with table of contents
   ├── getting_started.rst # Setup, build, and test instructions
   ├── mujoco_fundamentals.rst  # MuJoCo usage patterns
   ├── doc_infrastructure.rst   # This page
   └── api/
       └── index.rst       # Auto-generated C++ API reference

   Doxyfile                # (project root) Doxygen configuration

Building the Documentation
--------------------------

From the ``docs/`` directory:

.. code-block:: bash

   make html

This runs Doxygen first (to generate XML from C++ sources), then Sphinx (to
build the HTML site). The output is written to ``docs/_build/html/``.

To view locally:

.. code-block:: bash

   python3 -m http.server -d _build/html 8080
   # Open http://localhost:8080

To clean all build artefacts:

.. code-block:: bash

   make clean

Adding a New Page
-----------------

1. Create a new ``.rst`` file in ``docs/`` (e.g., ``my_topic.rst``).
2. Add a title using reStructuredText heading syntax::

      My Topic
      ========

3. Register it in ``index.rst`` under the appropriate ``toctree`` directive::

      .. toctree::
         :maxdepth: 2
         :caption: User Guide

         getting_started
         mujoco_fundamentals
         my_topic

4. Run ``make html`` and verify the page appears in the navigation.

C++ API Documentation
---------------------

API docs are generated automatically from source code comments via Doxygen +
Breathe:

- **Doxygen** reads ``include/`` and ``src/``, extracts documentation from
  comments (``///`` or ``/** */``), and writes XML to
  ``docs/_build/doxygen/xml/``.
- **Breathe** makes that XML available as Sphinx directives. In any ``.rst``
  file, you can pull in documentation for a namespace, class, or function::

      .. doxygennamespace:: morphology
         :members:

      .. doxygenclass:: brain::NeuralController
         :members:

      .. doxygenfunction:: sim::evaluate

The API reference page (``docs/api/index.rst``) uses namespace-level directives
to generate a complete reference. As new code is written and documented with
comments, it appears automatically on the next ``make html``.

CI Integration
--------------

The GitHub Actions CI workflow (``.github/workflows/ci.yml``) builds the
documentation on every push and pull request. This ensures that documentation
stays buildable and that Doxygen/Sphinx warnings are caught early.
