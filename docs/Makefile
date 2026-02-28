# Minimal makefile for Sphinx documentation
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build
DOXYGENDIR    = _build/doxygen

.PHONY: help clean doxygen html

help:
	@echo "  make doxygen  – run Doxygen to generate XML for Breathe"
	@echo "  make html     – build HTML docs (runs Doxygen first)"
	@echo "  make clean    – remove all build artefacts"

clean:
	rm -rf $(BUILDDIR)

doxygen:
	@mkdir -p $(DOXYGENDIR)
	cd .. && doxygen Doxyfile

html: doxygen
	$(SPHINXBUILD) -b html $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS)
	@echo ""
	@echo "Build finished. Open $(BUILDDIR)/html/index.html to view."
