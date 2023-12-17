This file is for reference uses.

# Common Emojis
🏆 🏗️ 📌 🔬 🧰 ⚙ 🚀 📉 🎙️ 📸 🧠 🤖 🥷🏻 🌐 📧 ✒️ 🏘️ 🦾 🔗 🖋️ 📚️ ☁️  🕹️ 💻 🎮️ 📜 🧋 📚️

# Common Patterns
-> [中文](/SKILLSETS/MATH/POSSIBILITY&STATISTICS/possibility&statistics-cn.md)
# POSSIBILITY AND STATISTICS
TO BE FILLED

-> [English](/SKILLSETS/MATH/POSSIBILITY&STATISTICS/possibility&statistics.md)
# 概率与统计
待更新

# Docsify Plugins & Configurations

## PDF Embedding
put the following code in the `index.html` file of the docsify project
the first line is the plugin
the second / third line is the format of the pdf file

```
<!-- PDFObject.js is a required dependency of this plugin -->
<script src="//cdnjs.cloudflare.com/ajax/libs/pdfobject/2.1.1/pdfobject.min.js"></script>

<!-- This is the source code of the pdf embed plugin -->
<script src="path-to-file/docsify-pdf-embed.js"></script>

<!-- or use this if you are not hosting the file yourself -->
<script src="//unpkg.com/docsify-pdf-embed-plugin/src/docsify-pdf-embed.js"></script>
```

in the position to use the pdf, insert
    
```
```pdf
    <path to the pdf file>
```
```
example:
    
```
```pdf
    /docs/DEVENV/OS/Windows/CMD_Cheat_Sheet.pdf
```
```

or 
```
```pdf
    https://www.cs.columbia.edu/~sedwards/classes/2015/1102-fall/Command%20Prompt%20Cheatsheet.pdf
```
```

## PDF Embedding

!!! tip

    If you don't have prior experience with Python, we recommend reading
    [Using Python's pip to Manage Your Projects' Dependencies], which is a
    really good introduction on the mechanics of Python package management and
    helps you troubleshoot if you run into errors.


:fontawesome-brands-youtube:{ style="color: #EE0F0F" }
__[How to set up Material for MkDocs]__ by @james-willett – :octicons-clock-24:
15m – Learn how to create and host a documentation site using Material for
MkDocs on GitHub Pages in a step-by-step guide.

??? question "How to add plugins to the Docker image?"

    Material for MkDocs only bundles selected plugins in order to keep the size
    of the official image small. If the plugin you want to use is not included,
    you can add them easily:

    === "Material for MkDocs"

        Create a `Dockerfile` and extend the official image:

        ``` Dockerfile title="Dockerfile"
        FROM squidfunk/mkdocs-material
        RUN pip install mkdocs-macros-plugin
        RUN pip install mkdocs-glightbox
        ```

## Site structure

Set up and customize the structure of your documentation by configuring the
header and footer to your taste, choosing among many modes of navigation,
setting up site search, and more.

<div class="grid cards" markdown>

- :fontawesome-solid-earth-americas: __[Language]__ – Choose out of the 60+ supported languages or add a new one
- :material-page-layout-sidebar-left: __[Navigation]__ – Create a clear, concise, and comprehensive navigation structure
- :material-page-layout-header: __[Header]__ – Customize the behavior of the header, add an announcement bar
- :material-page-layout-footer: __[Footer]__ – Add links to your social media profiles or websites in the footer
- :material-tab-search: __[Search]__ – Set up and configure search, running entirely in the user's browser
- :material-tag-plus-outline: __[Tags]__ – Categorize your pages with tags and group related pages

</div>

  [Language]: changing-the-language.md
  [Navigation]: setting-up-navigation.md
  [Header]: setting-up-the-header.md
  [Footer]: setting-up-the-footer.md
  [Search]: setting-up-site-search.md
  [Tags]: setting-up-tags.md