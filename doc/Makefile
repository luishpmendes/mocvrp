# Build the MO-CVRP formulation document.
TEX  := mocvrp.tex
PDF  := $(TEX:.tex=.pdf)

.PHONY: all clean distclean

all: $(PDF)

$(PDF): $(TEX)
	latexmk -pdf -halt-on-error -interaction=nonstopmode $(TEX)

clean:
	latexmk -c $(TEX)

distclean:
	latexmk -C $(TEX)
