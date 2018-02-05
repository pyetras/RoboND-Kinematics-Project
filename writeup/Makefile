../writeup.pdf : writeup.md eisvogel.tex
	pandoc writeup.md -o ../writeup.pdf --from markdown --template eisvogel.tex --listings

eisvogel.tex:
	wget https://raw.githubusercontent.com/Wandmalfarbe/pandoc-latex-template/master/eisvogel.tex

all: ../writeup.pdf

clean :
	rm -rf ../writeup.pdf