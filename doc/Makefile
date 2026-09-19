# Build the MO-CVRP formulation document.
TEX  := mocvrp.tex
BIB  := references.bib
PDF  := $(TEX:.tex=.pdf)

.PHONY: all clean distclean

all: $(PDF)

$(PDF): $(TEX) $(BIB)
	latexmk -pdf -halt-on-error -interaction=nonstopmode $(TEX)

clean:
	latexmk -c $(TEX)

distclean:
	latexmk -C $(TEX)
