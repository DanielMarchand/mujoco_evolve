# Configuration file for the Sphinx documentation builder.
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import subprocess

# -- Project information -----------------------------------------------------
project = "MuJoCo Evolve"
copyright = "2026, Daniel"
author = "Daniel"
release = "0.1.0"

# -- General configuration ---------------------------------------------------
extensions = [
    "breathe",              # Doxygen <-> Sphinx bridge for C++ API docs
    "sphinx_copybutton",    # "Copy" button on code blocks
    "sphinxcontrib.katex",  # Math rendering via KaTeX
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output -------------------------------------------------
html_theme = "furo"
html_title = "MuJoCo Evolve"
html_theme_options = {
    "navigation_with_keys": True,
}

# -- Breathe configuration ---------------------------------------------------
breathe_projects = {
    "mujoco_evolve": os.path.join(os.path.dirname(__file__), "_build", "doxygen", "xml"),
}
breathe_default_project = "mujoco_evolve"
breathe_default_members = ("members", "undoc-members")

# -- KaTeX configuration -----------------------------------------------------
katex_prerender = False
