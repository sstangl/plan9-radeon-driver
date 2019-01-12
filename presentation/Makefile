
TEXFILES = $(wildcard *.tex)

all: radeon.pdf

%.pdf: %.tex $(TEXFILES)
	pdflatex $*

clean:
	rm -f *.pdf *.aux *.log
