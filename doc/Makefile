all: pdf

pdf:
	pdflatex main
	pdflatex main
	pdflatex main
	pdflatex formulas
	pdflatex formulas
	pdflatex formulas

clean:
	rm -f *.aux *.log *.bbl *.blg *.toc *.lof *.lot *.out
	rm -f head/*.aux head/*.log
	rm -f main/*.aux main/*.log
	rm -f tail/*.aux tail/*.log
