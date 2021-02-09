(TeX-add-style-hook
 "mainDocument"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("xcolor" "table" "usenames" "dvipsnames") ("circuitikz" "siunitx") ("mdframed" "framemethod=TikZ")))
   (TeX-run-style-hooks
    "latex2e"
    "latex/labBookMechatronics/BBBlueIntroC"
    "article"
    "art10"
    "fancyhdr"
    "lastpage"
    "hyperref"
    "amsmath"
    "amssymb"
    "bm"
    "mathrsfs"
    "mathtools"
    "graphicx"
    "caption"
    "subfigure"
    "xcolor"
    "setspace"
    "array"
    "float"
    "verbatim"
    "epstopdf"
    "booktabs"
    "siunitx"
    "tikz"
    "circuitikz"
    "todonotes"
    "enumerate"
    "mdframed")
   (TeX-add-symbols
    '("midskip" 0)
    '("vsmallskip" 0)
    "name"
    "coursetitle"
    "dept"
    "college"
    "uname"
    "labTitle"
    "myCktGrid")
   (LaTeX-add-environments
    '("prelab" LaTeX-env-args ["argument"] 1)
    "example"
    "exercise"
    "definition")
   (LaTeX-add-bibliographies
    "bib/refsBooksTRTheses")
   (LaTeX-add-counters
    "prelab"))
 :latex)

