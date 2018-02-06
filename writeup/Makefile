../writeup.pdf : writeup.md eisvogel-notables.tex
	pandoc writeup.md -o ../writeup.pdf --from markdown --template eisvogel-notables.tex --listings

eisvogel-notables.tex: eisvogel.tex
	sed 's/\\rowcolors{3}{}{table-row-color100}//g' < eisvogel.tex > eisvogel-notables.tex

eisvogel.tex:
	wget https://raw.githubusercontent.com/Wandmalfarbe/pandoc-latex-template/master/eisvogel.tex

all: ../writeup.pdf

clean :
	rm -rf ../writeup.pdf eisvogel-notables.tex
