SRC_SVG := $(wildcard images/*.svg)
SRC_EPS := $(wildcard images/*.eps)
OBJECTS_EPS := $(SRC_SVG:.svg=.eps)
OBJECTS_PDF := $(OBJECTS_EPS:.eps=.pdf) $(SRC_EPS:.eps=.pdf)

.PHONY: all clean
all: template.pdf
	@:
clean:
	-rm -f template.pdf template.out template.aux template.toc template.log
	-rm -f $(OBJECTS_EPS) $(OBJECTS_PDF)

%.pdf: %.tex $(wildcard *.tex) $(OBJECTS_EPS) $(OBJECTS_PDF)
	pdflatex $<
	pdflatex $<		# rerun to get the references right

images/%.eps: images/%.svg
	-inkscape $< --export-eps=$@
images/%.pdf: images/%.eps
	-epstopdf --outfile=$@ $<
