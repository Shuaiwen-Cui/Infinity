# LATEX

## 模板

### Elsevier
<div class="grid cards" markdown>

-   :simple-elsevier:{ .lg .middle } __Latex 模板__

    ---

    Elsevier CAS 模板.

    [:octicons-arrow-right-24: <a href="https://www.elsevier.com/researcher/author/policies-and-guidelines/latex-instructions" target="_blank"> 传送门 </a>](#)

    [:octicons-arrow-right-24: <a href="https://mirror.las.iastate.edu/tex-archive/macros/latex/contrib/els-cas-templates/doc/elsdoc-cas.pdf" target="_blank"> 文档 </a>](#)

</div>

## LATEX 环境

### 在线编辑器

#### Overleaf

<div class="grid cards" markdown>

-   :simple-overleaf:{ .lg .middle } __Overleaf__

    ---

    Overleaf 是一个协作式基于云的 LaTeX 编辑器，用于编写、编辑和发布科学文档。

    [:octicons-arrow-right-24: <a href="https://www.overleaf.com/" target="_blank"> 传送门 </a>](#)

    [:octicons-arrow-right-24: <a href="https://www.overleaf.com/learn" target="_blank"> 文档 </a>](#)

</div>

### 本地编辑器

#### VSCode

<div class="grid cards" markdown>

-   :simple-visualstudiocode:{ .lg .middle } __VSCode + Latex Workshop__

    ---

    Visual Studio Code 是由 Microsoft 开发的用于 Windows、Linux 和 macOS 的源代码编辑器。

    配置 VSCode 中的 latex:

    (1) 在计算机上下载适当的 latex 编译器: Windows / Mac / Linux.

    (2) 安装 VSCode 和 Latex Workshop 扩展。

    (3) 默认配置对大多数用户来说应该足够了。

    (4) 自定义

    [:octicons-arrow-right-24: <a href="https://mathjiajia.github.io/vscode-and-latex/" target="_blank"> Windows </a>](#)

    [:octicons-arrow-right-24: <a href="https://hackmd.io/@x5758x/maclatex" target="_blank"> Mac </a>](#)

    [:octicons-arrow-right-24: <a href="https://github.com/shinyypig/latex-vscode-config" target="_blank"> VSCode 设置 </a>](#)
    

</div>

!!! tip
    代码->PDF: `Ctrl + Alt + J`
    
    PDF->代码: `Ctrl + 鼠标左键`

!!! tip
    插件 better bibtex 可以帮助更好的管理文献引用。
    其中citation key 可以设置为`[auth:lower]_[veryshorttitle:lower]_[year]`,记得选中所有论文然后refresh后导出bib文件。    

VSCODE SETTINGS 配置
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
