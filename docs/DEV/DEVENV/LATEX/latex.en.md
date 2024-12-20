# LATEX

## Template

### Elsevier
<div class="grid cards" markdown>

-   :simple-elsevier:{ .lg .middle } __Latex Template__

    ---

    Elsevier CAS template.

    [:octicons-arrow-right-24: <a href="https://www.elsevier.com/researcher/author/policies-and-guidelines/latex-instructions" target="_blank"> Portal </a>](#)

    [:octicons-arrow-right-24: <a href="https://mirror.las.iastate.edu/tex-archive/macros/latex/contrib/els-cas-templates/doc/elsdoc-cas.pdf" target="_blank"> Documentation </a>](#)

</div>

## LATEX Environment

### Online Editor

#### Overleaf

<div class="grid cards" markdown>

-   :simple-overleaf:{ .lg .middle } __Overleaf__

    ---

    Overleaf is a collaborative cloud-based LaTeX editor used for writing, editing and publishing scientific documents.

    [:octicons-arrow-right-24: <a href="https://www.overleaf.com/" target="_blank"> Portal </a>](#)

    [:octicons-arrow-right-24: <a href="https://www.overleaf.com/learn" target="_blank"> Documentation </a>](#)
    
</div>

### Local Editor

#### VSCode

<div class="grid cards" markdown>

-   :simple-visualstudiocode:{ .lg .middle } __VSCode + Latex Workshop__

    ---

    Visual Studio Code is a source code editor developed by Microsoft for Windows, Linux, and macOS.

    Configure latex in VSCode:

    (1) Download the appropriate latex compiler on your computer: Windows / Mac / Linux.

    (2) Install VSCode and Latex Workshop extension.

    (3) The default configuration should be sufficient for most users.

    (4) Customization

    [:octicons-arrow-right-24: <a href="https://mathjiajia.github.io/vscode-and-latex/" target="_blank"> Windows </a>](#)

    [:octicons-arrow-right-24: <a href="https://hackmd.io/@x5758x/maclatex" target="_blank"> Mac </a>](#)

    [:octicons-arrow-right-24: <a href="https://github.com/shinyypig/latex-vscode-config" target="_blank"> VSCode Set Up </a>](#)

</div>

!!! tip
    Code -> PDF: `Ctrl + Alt + J`
    
    PDF -> Code: `Ctrl + Mouse Left Click`

!!! tip
    Plugin better bibtext can help manage references better.
    The citation key can be setup as `[auth:lower]_[veryshorttitle:lower]_[year]`. Before exporting, remember to select all papers and refresh to export the bib file.

VSCODE SETTINGS CONFIGURATION
```text
    "latex-workshop.hover.preview.mathjax.extensions": [
        "boldsymbol"
    ],
    "latex-workshop.intellisense.package.enabled": true,
    "latex-workshop.latex.outDir": ".",
    "latex-workshop.latex.recipe.default": "lastUsed",
    "latex-workshop.mathpreviewpanel.cursor.enabled": true,
    "latex-workshop.message.error.show": false,
    "latex-workshop.message.warning.show": false,
    // "latex-workshop.view.pdf.invert": 1,
    // "latex-workshop.view.pdf.invertMode.enabled": "auto",
    "latex-workshop.latex.recipes": [
        {
            "name": "XeLaTeX",
            "tools": [
                "xelatexmk"
            ]
        },
        {
            "name": "PdfLaTeX",
            "tools": [
                "pdflatexmk"
            ]
        }
    ],
    "latex-workshop.latex.tools": [
        {
            "args": [
                "-synctex=1",
                "-pdfxe",
                "-interaction=nonstopmode",
                "-file-line-error",
                "-outdir=%OUTDIR%",
                "%DOC%"
            ],
            "command": "latexmk",
            "env": {},
            "name": "xelatexmk"
        },
        {
            "args": [
                "-synctex=1",
                "-pdf",
                "-interaction=nonstopmode",
                "-file-line-error",
                "-outdir=%OUTDIR%",
                "%DOC%"
            ],
            "command": "latexmk",
            "env": {},
            "name": "pdflatexmk"
        }
    ],
```
