# Tech Blog Using MkDocs and Material Theme

## Introduction

For tech blog, one may don't want to use the traditional blog platform, such as WordPress, Blogger, etc. Instead, one may want to use a static site generator, which is much lightweight and easy to use. Popular static site generators include:

- [Jekyll](https://jekyllrb.com/)

- [Hugo](https://gohugo.io/)

- [Hexo](https://hexo.io/)

- [MkDocs](https://www.mkdocs.org/) (My Pick)

### What Is MkDocs?

MkDocs is a fast, simple and downright gorgeous static site generator that's geared towards building project documentation. Documentation source files are written in Markdown, and configured with a single YAML configuration file. It features a built-in development server that reloads your changes as you edit the source files, so you can see your changes instantly. MkDocs is available on [PyPI](https://pypi.org/project/mkdocs/) and can be installed through pip.

### What Is Material Theme?

Material is a theme for [MkDocs](https://www.mkdocs.org/), an excellent static site generator geared towards project documentation. It is built using Google's [Material Design](https://material.io/) guidelines. Material has been designed to be beautiful and easy to use. It comes with built-in support for [Material Design icons](https://material.io/icons/), [Material Design colours](https://material.io/guidelines/style/color.html), and [Roboto](https://fonts.google.com/specimen/Roboto) fonts. Material is also fully responsive, meaning it looks great on desktop as well as mobile devices.

## How To Use MkDocs?

### Installation

MkDocs is available from [PyPI](https://pypi.org/project/mkdocs/) and can be installed through pip:

```bash
pip install mkdocs
```

### Getting Started

Getting started is super easy. 

#### Step 1: Start a new project

```bash
mkdocs new my-project
```

This step will create a new project directory named `my-project` containing a single configuration file named `mkdocs.yml` and a folder named `docs` that will contain your documentation source files. Inside the `docs` directory, a new file named `index.md` will already exist. Open this file in your text editor. The folder structure should look like this:

```bash
my-project/
  mkdocs.yml
  docs/
    index.md
```

#### Step 2: Preview your site

```bash
cd my-project
mkdocs serve
```
This step will start the built-in dev-server, which will reload your documentation whenever anything changes. Open up http://127.0.0.1:8000/ in your browser, and you'll see the default home page being displayed. The dev-server also supports auto-reloading, and will rebuild your documentation whenever anything in the configuration file, documentation directory, or theme directory changes.

#### Step 3: Deploy

```bash
mkdocs build
```

This step is to build the documentation site. The documentation site will be located in the `site` directory, which can be copied to any standard web server for hosting.

!!! tip
    Note that, better build the website in the server rather than locally. That means, when uploading the website to the server, don't upload the `site` directory. For example, you can use gitignore to ignore the `site` directory. Then, in the server, in corresponding directory, run `mkdocs build` to build the website. 

### Configuration

The configuration file is named `mkdocs.yml`, and is located in the root of your project. The configuration file is a YAML file, with a simple structure. The configuration file contains a list of pages (in the `pages` key), a list of options (in the `site_name` key), and a list of pages to be excluded from the build (in the `exclude` key).

```yaml
site_name: MkLorum
pages:
- Home: index.md
- About: about.md
- License: license.md
```

!!! tip
    For this part, if you use the material them later, you can directly use the template `mkdocs.yml` file in the `material` directory. Follow the instructions from Material for MkDocs to modify the configuration.

### Writing Documentation

The documentation files are written in Markdown, and configured with a single YAML configuration file. The documentation files are stored in the `docs` directory by default, but you can customize this using the `docs_dir` configuration parameter.

## How To Use Material Theme?

refer to [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/), there is a very detailed documentation. 

### Installation

Material for MkDocs is available from [PyPI](https://pypi.org/project/mkdocs-material/) and can be installed through pip:

```bash
pip install mkdocs-material
```

### Configuration

This part should be paid attention to. I use my own configuration file as an example. Refer to the comments in the configuration file for details.

```yaml
# Project information
site_name: Eureka! # site name
site_url: https://localhost:8000/ # build and run on server, to visit it, type in the server's IP address and port number like this: xxx.xxx.xxx.xxx:8000
site_author: Shuaiwen Cui # author name
site_description: >- # site description
  Welcome to Shaun's rabbit hole. This site serves as a personal knowledge base for me to record my thoughts and ideas. It is also a place for me to share my knowledge and experience with the world. I hope you find something useful here. 

# Repository - if you want to share your website repo, you can add this part
repo_name: Shuaiwen-Cui/Infinity # repo name
repo_url: https://github.com/Shuaiwen-Cui/Infinity.git/ # repo url

# Copyright - there will be a section in the left bottom showing the info below
copyright: Copyright &copy; 2023 ~ now | 🚀 Shuaiwen Cui (Shaun)

# Configuration
theme:
  custom_dir: material/overrides # custom directory for overriding theme templates, for certain pages, you may don't want to use the default template, you can create a new template in this directory and use it. 
  name: material # theme name, material is the most popular one
  logo: static/images/logo.png # logo
  language: en # default language
  features: # features to be used    
    - announce.dismiss # announcement bar dismiss button
    - content.action.edit # edit button
    - content.action.view # view button
    - content.code.annotate # code annotation
    - content.code.copy # copy code button
    # - content.code.select # select code button
    # - content.tabs.link # link tabs to sections
    - content.tooltips # tooltips
    # - header.autohide # auto hide header
    - navigation.expand # expand the side navigation bar by default
    - navigation.footer # footer navigation
    - navigation.indexes # documents can be directly attached to sections, which is particularly useful for providing overview pages
    - navigation.instant # instant navigation
    - navigation.instant.prefetch # for large file
    - navigation.instant.progress # loading progress bar
    - navigation.path # breadcrumb path on top of page
    # - navigation.prune # only build the visible part of the navigation tree
    - navigation.sections # navigation - top level will render as sections
    - navigation.tabs # navigation - top level will render as tabs
    - navigation.tabs.sticky # tabs stick to the top of the page
    - navigation.top # top navigation
    - navigation.tracking # navigation tracking
    - search.highlight # search result highlight
    - search.share # share search results
    - search.suggest # search suggestions
    - toc.follow # table of content follow scroll
    # - toc.integrate # toc is merged into the left navigation bar
  palette:
    - media: "(prefers-color-scheme)" # color scheme
      scheme: slate
      primary: black
      accent: indigo
      toggle:
        icon: material/link
        name: Switch to light mode
    - media: "(prefers-color-scheme: light)" # color scheme: light
      scheme: default
      primary: indigo
      accent: indigo
      toggle:
        icon: material/toggle-switch
        name: Switch to dark mode
    - media: "(prefers-color-scheme: dark)" # color scheme: dark
      scheme: slate
      primary: black
      accent: indigo
      toggle:
        icon: material/toggle-switch-off
        name: Switch to system preference
  font: # font
    text: Roboto
    code: Roboto Mono
  favicon: assets/favicon.png # favicon
  icon: # icons used
    logo: logo
    previous: fontawesome/solid/angle-left
    next: fontawesome/solid/angle-right
    tag:
      default-tag: fontawesome/solid/tag
      hardware-tag: fontawesome/solid/microchip
      software-tag: fontawesome/solid/laptop-code

# Plugins
plugins:
  - tags # tags
  - blog # blog
  - rss: # rss
      match_path: blog/posts/.* 
      date_from_meta:
        as_creation: date
      categories:
        - categories
        - tags 
  # - social
  - search: # search
      separator: '[\s\u200b\-_,:!=\[\]()"`/]+|\.(?!\d)|&[lg]t;|(?!\b)(?=[A-Z][a-z])'
  - minify: # minify
      minify_html: true
  # - privacy
  - i18n: # internationalization for language switch
      docs_structure: suffix
      fallback_to_default: true
      reconfigure_material: true
      reconfigure_search: true
      languages: # This part should be really careful. For now, I cannot do stay on the same page after switching language. There should be more elegant way to do this.
        - locale: en
          default: true
          name: English
          build: true
          # site_name: Infinity
        - locale: zh
          name: 简体中文
          build: true
          nav_translations: # note that all titles and subtitles in the navigation bar must be translated need to be translated can be put here, but with no indentation
            HOME: 首页
            ABOUT: 关于
            SPONSORSHIP: 赞助
            CS: 计算机
            CODING: 编程
            EMBEDDED-SYS: 嵌入式系统
            DSP: 数字信号处理
            PERCEPTION: 感知
            ACTUATION: 执行
            IOT: 物联网
            CLOUD: 云
            CLOUD-TECH: 云技术
            HANDS-ON: 上手实践
            Have A Server: 拥有一台服务器
            Server Configuration: 服务器配置
            AI: 人工智能
            RESEARCH: 研究
            PROJECT: 项目
# # Hooks
# hooks:
#   - material/overrides/hooks/shortcodes.py
#   - material/overrides/hooks/translations.py

# Additional configuration
extra:
  generator: false # show the generator in the footer
  status:
    new: Recently added
    deprecated: Deprecated
  analytics:
    provider: google
    property: !ENV GOOGLE_ANALYTICS_KEY
    feedback: # feedback form
      title: Was this page helpful?
      ratings:
        - icon: material/thumb-up-outline
          name: This page was helpful
          data: 1
          note: >-
            Thanks for your feedback!
        - icon: material/thumb-down-outline
          name: This page could be improved
          data: 0
          note: >- 
            Thanks for your feedback! Help us improve this page by
            using our <a href="..." target="_blank" rel="noopener">feedback form</a>.
  # alternate:
  #   - name: English
  #     link: /en/ 
  #     lang: en
  #   - name: Chinese
  #     link: /zh/
  #     lang: zh
  social: # social media links at the right bottom
    - icon: fontawesome/solid/house
      link: http://www.cuishuaiwen.com/
    - icon: fontawesome/brands/github
      link: https://github.com/Shuaiwen-Cui
    - icon: fontawesome/brands/linkedin
      link: https://www.linkedin.com/in/shaun-shuaiwen-cui/
    - icon: fontawesome/brands/researchgate
      link: https://www.researchgate.net/profile/Shuaiwen-Cui
    - icon: fontawesome/brands/orcid
      link: https://orcid.org/0000-0003-4447-6687
    - icon: fontawesome/brands/twitter
      link: https://twitter.com/ShuaiwenC
  tags: # tags - corresponding to the tags plugin
    Default: default-tag
    Hardware: hardware-tag
    Software: software-tag
  # consent:
  #   title: Cookie consent
  #   description: >- 
  #     We use cookies to recognize your repeated visits and preferences, as well
  #     as to measure the effectiveness of our documentation and whether users
  #     find what they're searching for. With your consent, you're helping us to
  #     make our documentation better.

# Extensions
markdown_extensions: # markdown extensions
  - abbr
  - admonition
  - attr_list
  - def_list
  - footnotes
  - md_in_html
  - toc:
      permalink: true
  - pymdownx.arithmatex:
      generic: true
  - pymdownx.betterem:
      smart_enable: all
  - pymdownx.caret
  - pymdownx.details
  - pymdownx.emoji:
      emoji_generator: !!python/name:material.extensions.emoji.to_svg
      emoji_index: !!python/name:material.extensions.emoji.twemoji
  - pymdownx.highlight:
      anchor_linenums: true
      line_spans: __span
      pygments_lang_class: true
  - pymdownx.inlinehilite
  - pymdownx.keys
  - pymdownx.magiclink:
      normalize_issue_symbols: true
      repo_url_shorthand: true
      user: squidfunk
      repo: mkdocs-material
  - pymdownx.mark
  - pymdownx.smartsymbols
  - pymdownx.snippets:
      auto_append:
        - includes/mkdocs.md
  - pymdownx.superfences:
      custom_fences:
        - name: mermaid
          class: mermaid
          format: !!python/name:pymdownx.superfences.fence_code_format
  - pymdownx.tabbed:
      alternate_style: true
      combine_header_slug: true
      slugify: !!python/object/apply:pymdownx.slugs.slugify
        kwds:
          case: lower
  - pymdownx.tasklist:
      custom_checkbox: true
  - pymdownx.tilde

# Page Navigation Tree
nav: # navigation tree - please follow my way to do this, otherwise, it may not work. the quotes can be omitted. The dot and slash in the beginning can also be omitted. ("./HOME/about.md" or Home/about.md) 
# note that here, the file name is like filename.md, but in the folders, it is actually named as filename.en.md and filename.zh.md. This is because, in the configuration file, I set the default language as English. So, if you want to add a new page, you need to add two files, one is filename.en.md, the other is filename.zh.md.
  - HOME: 
      - "index.md"
      - ABOUT: "./HOME/about.md"
      - SPONSORSHIP: "./HOME/sponsorship.md"
  - CS: 
      - "./CS/CS.md"
  - CODING: 
      - "./CODING/coding.md"
  - EMBEDDED-SYS: 
      - "./EMBEDDED-SYS/embedded-sys.md"
  - DSP: 
      - "./DSP/dsp.md"
  - PERCEPTION: 
      - "./PERCEPTION/perception.md"
  - ACTUATION: 
      - "./ACTUATION/actuation.md"
      - ROS: "./ACTUATION/ROS/ros.md"
  - IOT: 
      - "./IOT/iot.md"
  - CLOUD: 
      - "./CLOUD/cloud.md"
      - CLOUD-TECH: "./CLOUD/CLOUD-TECH/cloud-tech.md"
      - HANDS-ON:
          - Have A Server: "./CLOUD/HANDS-ON/001-HAVE-A-SERVER/have-a-server.md"
          - Server Configuration: "./CLOUD/HANDS-ON/002-SERVER-CONFIG/server-config.md"
          - Get A Domain Name: "./CLOUD/HANDS-ON/003-DOMAIN-NAME/domain-name.md"
  - AI: 
      - "./AI/ai.md"
  - RESEARCH: 
      - "./RESEARCH/research.md"
  - PROJECT: 
      - "./PROJECT/project.md"
      - TECH-BLOG: "./PROJECT/TECH-BLOG/mkdocs_and_material.md"
```
This configuration file should help you build a gorgeous website like you see now.

### Writing Documentation

For a tech blog, the contents matter the most, but I cannot help you with that. However, I can give a useful tip for you to make your article more beautiful.

!!! tip
    you can check the material theme documentation to study their markdown syntax.  

    Here are some useful syntax:

    - Note
    ```markdown
    !!! note
        This is a note.
    ```

    - Tip
    ```markdown
    !!! tip
        This is a tip.
    ```

    - Warning
    ```markdown
    !!! warning
        This is a warning.
    ```

    - Danger
    ```markdown
    !!! danger
        This is a danger.
    ```

    - Success
    ```markdown
    !!! success
        This is a success.
    ```
    - Info
    ```markdown
    !!! info
        This is an info.
    ```
    - Quote
    ```markdown
    !!! quote
        This is a quote.
    ```
    - Question
    ```markdown
    ??? question "What is the meaning of life, the universe, and everything?"
        42.
    ```

- Note

!!! note
    This is a note.

- Tip


!!! tip
    This is a tip.

- Warning


!!! warning
    This is a warning.

- Danger


!!! danger
    This is a danger.

- Success


!!! success
    This is a success.

- Info


!!! info
    This is an info.

- Quote


!!! quote
    This is a quote.

- Question


??? question "What is the meaning of life, the universe, and everything?"
    Nothing.

## Upload And Deploy - Method 1 - Direct Upload To Server

### Upload

```bash
scp -r <local directory> <username>@<server address>:<remote directory>
```

### Deploy

```bash
cd <remote directory>
mkdocs build
```

### Mount To Nginx

Before mounting to Nginx, you need to install Nginx first.

```bash
sudo apt-get install nginx
```
Then, you need to modify the configuration file of Nginx.

```bash
sudo vim /etc/nginx/sites-available/default
```

Enter the following file, press `i` to enter insert mode, and modify the file as follows:

```bash
server {
    listen 80; # by default, the port is 80 # if you use other port (e.g., 8000), you need to specify it here. Accordingly, when you visit the website, you need to type in the port number like this: xxx.xxx.xxx.xxx:8000
    server_name <server address>; # name as you like
    root <remote directory>/site; # the site folder is where the built website is located
    index index.html index.htm;
    location / {
        try_files $uri $uri/ =404;
    }
}
```
Then, press `esc` to exit insert mode, and type in `:wq` to save and exit.

Finally, restart Nginx.

```bash
sudo nginx -t # test
sudo systemctl restart nginx # restart nginx
```

## Upload And Deploy - Method 2 - Upload To GitHub And Deploy To Server (Recommended)

If you are familiar with git & github, this method is recommended.

I assume you have already link your local repo to the remote repo on GitHub. If not, please refer to [GitHub Docs](https://docs.github.com/en/github/getting-started-with-github/create-a-repo) to create a repo and link it to your local repo.

!!! tip
    Note that, better build the website in the server rather than locally. That means, when uploading the website to the server, don't upload the `site` directory. For example, you can use gitignore to ignore the `site` directory. Then, in the server, in corresponding directory, run `mkdocs build` to build the website.

    To use a gitignore file, you can create a file named `.gitignore` in the root directory of your repo. Then, you can add the following content to the file:

    ```bash
    # Ignore site directory
    site/
    ```

### Upload

```bash
git add .
git commit -m "update"
git push origin master # or it can be 'git push origin main'
```
If you use github pages to host your website, you can use optionally use the following command to push the website to the gh-pages branch.

```bash
mkdocs gh-deploy
```

!!! tip
    For me, I have a .sh file to do this. You can refer to the `upload.sh` file in this repo. You can modify it as you like. Then, you can use the following command to upload the website to the server.

    ```bash
    ./upload.sh # for Windows
    ```

    or

    ```bash
    sh upload.sh # for Linux
    ```

    or

    ```bash
    bash upload.sh # for Linux
    ```
    
### Deploy

On the server, you need to pull the repo and build the website.

If this is the first time you pull the repo, you need to clone the repo first.

```bash
git clone <repo url>
```

Otherwise, you can pull the repo.

```bash
git pull
```

Then, enter the repo directory, and build the website.

```bash
cd <repo directory>
```

Then, build the website.

```bash
mkdocs build
```

Finally, mount to Nginx. Please refer to the previous section in Method 1.

## The End

Enjoy your website!
