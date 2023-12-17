# 使用 MkDocs 和 Material 主题搭建技术博客

## 介绍

对于技术博客而言，我们或许不需要炫酷的动画效果，但是我们需要一个简洁、易于维护的博客框架。本文将介绍如何使用 [MkDocs](https://www.mkdocs.org/) 和 [Material 主题](https://squidfunk.github.io/mkdocs-material/)搭建一个简洁、易于维护的技术博客。

除了MkDocs和Material主题，市面上还有其他选择：

- [docsify](https://docsify.js.org/#/)

- [VuePress](https://vuepress.vuejs.org/zh/)

- [Docusaurus](https://docusaurus.io/)

- [GitBook](https://www.gitbook.com/)

- [Hexo](https://hexo.io/zh-cn/)

- [Jekyll](https://jekyllrb.com/)

- [Hugo](https://gohugo.io/)

- [Gatsby](https://www.gatsbyjs.com/)

- [DocFX](https://dotnet.github.io/docfx/)

- [Sphinx](https://www.sphinx-doc.org/en/master/)

### MkDocs 是什么？

MkDocs 是一个用于构建项目文档的工具，它可以将 Markdown 文件转换为静态网站。MkDocs 由 Python 编写，使用 [Python-Markdown](https://python-markdown.github.io/) 解析 Markdown 文件，使用 [Pygments](https://pygments.org/) 语法高亮代码，使用 [Jinja2](https://jinja.palletsprojects.com/en/2.11.x/) 模板引擎渲染 Markdown 文件。

### Material 主题是什么？

Material 主题是一个基于 [Material Design](https://material.io/) 的 MkDocs 主题，它提供了一个简洁、现代的界面，支持多种语言，支持自定义主题颜色，支持搜索、导航、侧边栏、标签、分类、评论、Google Analytics、Disqus、多种插件等功能。

## 如何使用MkDocs？

显然MkDocs可以脱离Material主题单独使用，但是Material主题依赖于MkDocs，所以我们需要先安装MkDocs。

### 安装MkDocs

MkDocs可以通过pip安装。

```bash
pip install mkdocs
```

如果国内网络环境不好，可以使用清华大学的镜像源。

```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple mkdocs
```

### 快速上手

安装完成后，我们可以快速构建一个网站。

#### 第一步：创建项目

```bash
mkdocs new my-project
```

这一步将会在当前目录下创建一个名为`my-project`的文件夹，该文件夹包含以下内容：

```bash
my-project/
├── docs/
│   └── index.md
└── mkdocs.yml
```

其中`docs`文件夹用于存放Markdown文件，`mkdocs.yml`文件用于配置MkDocs。Index.md是网站的首页，我们可以在其中添加一些内容。

#### 第二步：启动网站

```bash
cd my-project
mkdocs serve
```

这一步将会启动一个本地服务器，我们可以在浏览器中访问`http://127.0.0.1:8000/`查看网站。这一步使用了MkDocs的内置服务器，每次修改完，点击保存，网站就会自动刷新。

#### 第三步：构建网站

```bash
mkdocs build
```

这一步将会在`site`文件夹下生成一个静态网站，我们可以将该文件夹下的内容部署到服务器上。

!!! tip
    请注意，最好只上传原始的文件至服务器，然后在服务器上构建网站，因为MkDocs的版本可能会发生变化，如果本地构建的网站上传至服务器，可能会导致网站无法正常显示。如果你使用的是Git，可以在`.gitignore`文件中添加`site`，这样就不会将`site`文件夹上传至服务器。

### 配置MkDocs

MkDocs的配置文件是`mkdocs.yml`，我们可以在其中配置网站的名称、描述、主题、导航栏、侧边栏、插件等。

```yaml
site_name: My Project
site_description: A short description of my project.
site_author: Your Name
site_url: https://example.com
site_dir: site
site_favicon: images/favicon.ico
# ...
```

!!! tip
    这一部分可以暂时按下不动，强烈建议使用material主题，直接基于material主题模板进行配置修改。

### 写作

写作文件的格式是Markdown，我们可以在`docs`文件夹下创建Markdown文件，然后在`mkdocs.yml`中配置导航栏和侧边栏。默认的文件存放位置是`docs`，但是我们可以通过`docs_dir`配置文件存放位置。

## 如何使用Material主题？

可以参考[Material主题文档](https://squidfunk.github.io/mkdocs-material/)，其中有极为详细的使用说明。该文档本身就是使用Material主题搭建的，可以作为参考，非常值得参考学习。

!!! info
    Material 主题的文档好看是好看，尤其是那个landing page，真的想要，但是那个landing page应该是成为sponsor 才能拿到资源，理论上来说可以自己写一个类似的，在对应的页面使用override，但是目前我没有这个水平，所以暂时先不写了。
    另外，没有找到好用的切换语言的参考，我目前无法做到停留在当前页面，切换语言后会跳转到首页，这个问题暂时无法解决。

### 安装Material主题

Material主题可以通过pip安装。

```bash
pip install mkdocs-material
```

如果国内网络环境不好，可以使用清华大学的镜像源。

```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple mkdocs-material
```

### 配置Material主题

这个部分应该是花费时间和心思最多的部分，因为Material主题提供了非常多的配置选项，我们可以根据自己的需求进行配置。具体可以参考我的备注。

```yaml
# 项目信息
site_name: Eureka! # 项目名称
site_url: https://localhost:8000/ # 我在nginx中使用的是8000端口，如果你使用的是80端口，可以直接写成https://localhost/。
site_author: Shuaiwen Cui # 作者
site_description: >- # 项目描述
  Welcome to Shaun's rabbit hole. This site serves as a personal knowledge base for me to record my thoughts and ideas. It is also a place for me to share my knowledge and experience with the world. I hope you find something useful here. 

# 代码仓库信息
repo_name: Shuaiwen-Cui/Infinity # 仓库名称
repo_url: https://github.com/Shuaiwen-Cui/Infinity.git/ # 仓库地址

# 版权信息
copyright: Copyright &copy; 2023 ~ now | 🚀 Shuaiwen Cui (Shaun)

# 配置
theme:
  custom_dir: material/overrides # 自定义文件夹，对于个别页面，如果你不想使用主题的默认样式，可以在这里进行修改，使用里面的文件覆盖主题的默认文件。具体可以参考material官方文档。
  name: material # 主题名称，Material已经是最优秀的选择了，相信我。
  logo: static/images/logo.png # logo 图片
  language: en # 默认语言
  features: # 功能  
    - announce.dismiss # 可以叉掉公告的功能
    - content.action.edit # 编辑按钮，似乎没啥用
    - content.action.view # 查看按钮，似乎没啥用
    - content.code.annotate # 代码注释，具体不清楚
    - content.code.copy # 复制代码按钮
    # - content.code.select # 选择代码按钮
    # - content.tabs.link # 链接标签
    - content.tooltips # 不太清楚呢这个
    # - header.autohide # 自动隐藏header
    - navigation.expand # 默认展开导航栏
    - navigation.footer # 底部导航栏
    - navigation.indexes # 索引按钮可以直接触发文件，而不是只能点击其下属选项浏览，这个功能可以给对应的section提供很好的预览和导航功能
    - navigation.instant # 瞬间加载
    - navigation.instant.prefetch # 预加载
    - navigation.instant.progress # 进度条
    - navigation.path # 导航路径， 目前好像没啥用
    # - navigation.prune # 只构建可见的页面
    - navigation.sections # 导航栏的section
    - navigation.tabs # 顶级索引被作为tab
    - navigation.tabs.sticky # tab始终可见
    - navigation.top # 开启顶部导航栏
    - navigation.tracking # 导航栏跟踪
    - search.highlight # 搜索高亮
    - search.share # 搜索分享
    - search.suggest # 搜索建议
    - toc.follow # 目录跟踪-页面右侧的小目录
    # - toc.integrate # 目录跟踪集成到左侧大目录中
  palette:
    - media: "(prefers-color-scheme)" # 主题颜色
      scheme: slate
      primary: black
      accent: indigo
      toggle:
        icon: material/link
        name: Switch to light mode
    - media: "(prefers-color-scheme: light)" # 浅色
      scheme: default
      primary: indigo
      accent: indigo
      toggle:
        icon: material/toggle-switch
        name: Switch to dark mode
    - media: "(prefers-color-scheme: dark)" # 深色
      scheme: slate
      primary: black
      accent: indigo
      toggle:
        icon: material/toggle-switch-off
        name: Switch to system preference
  font: # 字体，大概率不需要换
    text: Roboto
    code: Roboto Mono
  favicon: assets/favicon.png # 网站图标 似乎不需要管
  icon: # 一些用到的icon
    logo: logo
    previous: fontawesome/solid/angle-left
    next: fontawesome/solid/angle-right
    tag:
      default-tag: fontawesome/solid/tag
      hardware-tag: fontawesome/solid/microchip
      software-tag: fontawesome/solid/laptop-code

# Plugins
plugins:
  - tags # 标签功能插件
  - blog # 博客功能插件
  - rss: # rss订阅插件 - 不太懂是干嘛的目前
      match_path: blog/posts/.* 
      date_from_meta:
        as_creation: date
      categories:
        - categories
        - tags 
  # - social # 目前我开启会报错，还没研究透 
  - search: # 搜索插件
      separator: '[\s\u200b\-_,:!=\[\]()"`/]+|\.(?!\d)|&[lg]t;|(?!\b)(?=[A-Z][a-z])' # 分隔符
  - minify: # 压缩插件
      minify_html: true
  # - privacy # 隐私插件
  - i18n: # 多语言插件
      docs_structure: suffix # 抄来的，不太懂
      fallback_to_default: true # 抄来的，不太懂
      reconfigure_material: true # 抄来的，不太懂
      reconfigure_search: true # 抄来的，不太懂
      languages: # 多语言配置 - 需要小心一点
        - locale: en
          default: true # 默认语言
          name: English
          build: true # 是否构建
          # site_name: Infinity
        - locale: zh
          name: 简体中文
          build: true
          nav_translations: # 导航栏翻译，不可以有缩进
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
            Get A Domain Name: 获得一个域名
            AI: 人工智能
            RESEARCH: 研究
            PROJECT: 项目
# Hooks - 讲真，这个东西我还没搞懂
# hooks:
#   - material/overrides/hooks/shortcodes.py
#   - material/overrides/hooks/translations.py

# 额外配置项
extra:
  generator: false # 是否显示生成器
  status: # 不是很懂有什么用
    new: Recently added
    deprecated: Deprecated
  analytics: # 分析工具， 我反正没用到
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
  # alternate: # 由上面那个i18n插件提供的多语言功能，这个似乎就不需要了。 这个是官方文档的例子，但是官方没有提供很详细的例子，所以我也不知道怎么用。尤其是那个stay on page，我做不来。
  #   - name: English
  #     link: /en/ 
  #     lang: en
  #   - name: Chinese
  #     link: /zh/
  #     lang: zh
  social: # 社交媒体
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
  tags: # 自定义标签
    Default: default-tag
    Hardware: hardware-tag
    Software: software-tag
  # consent: # 征求同意 Cookie
  #   title: Cookie consent
  #   description: >- 
  #     We use cookies to recognize your repeated visits and preferences, as well
  #     as to measure the effectiveness of our documentation and whether users
  #     find what they're searching for. With your consent, you're helping us to
  #     make our documentation better.

# 扩展
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

# 导航树 - 请按照我的做法来做，否则可能无法正常工作。引号可以省略。开头的点和斜杠也可以省略 ("./HOME/about.md" 或 Home/about.md) 。注意，导航树这里的文件名是 filename.md 这样的，但在文件夹中，它实际上被命名为 filename.en.md 和 filename.zh.md。我猜测默认是英文，所以, index.en.md 和 index.md 是一样的。i18n插件会自动识别文件名，然后根据文件名的后缀来切换语言。所以，如果你想添加一个新页面，你需要添加两个文件，一个是 filename.en.md，另一个是 filename.zh.md。其中，filename.en.md 也可以被命名为 filename.md，但是 filename.zh.md 不能被命名为 filename.md，否则会导致无法识别。
nav: 
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
搞定这个配置文件后，你的页面应该就会像我的一样好看了。事实上，除了外观，所有的配置功能基本上都在这个配置文件中，所以你可以根据自己的需求进行修改。

### 写作内容

。

!!! tip
    可以参考material主题文档中的写作内容部分，非常多值得借鉴的内容  

    以下句法可以重点学习：

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

## 上传并部署到服务器 - 第一种方法 - 直接上传

### 上传

我们可以使用`scp`命令将网站上传至服务器。

```bash
scp -r <local directory> <username>@<server address>:<remote directory>
```

### 部署

```bash
cd <remote directory>
mkdocs build
```

### 挂载到nginx

首先如果你没有安装nginx，可以使用以下命令安装。

```bash
sudo apt-get install nginx
```

然后进入nginx的配置文件夹。

```bash
sudo vim /etc/nginx/sites-available/default
```

进入该配置文件，按“i”进入编辑模式，然后将以下内容复制进去。


```bash
server {
    listen 80; # 默认为80端口，如果你使用的是80端口，可以直接写成https://localhost/。我使用的是8000端口，访问的时候对应的地址是https://localhost:8000/。
    server_name <server address>; # 随便
    root <remote directory>/site; # 找到site文件夹
    index index.html index.htm;
    location / {
        try_files $uri $uri/ =404;
    }
}
```
然后按“esc”退出编辑模式，然后输入“:wq”保存并退出。

最后重启nginx。

```bash
sudo nginx -t # test
sudo service nginx restart
```

## 上传并部署到服务器 - 第二种方法 - 使用Git & Github 并部署至服务器 （推荐）

如果你熟悉Git和Github，那么这种方法会更加方便。我们可以在本地使用Git管理网站，然后将网站上传至Github，然后在服务器上使用Git拉取网站，然后部署。

我假设你的本地库和远程库都已经建立好了，并且已经关联好了，如果没有建立好，可以参考[GitHub Docs](https://docs.github.com/en/github/getting-started-with-github/create-a-repo)。

!!! tip
    注意，最好在服务器上构建网站，因为MkDocs的版本可能会发生变化，如果本地构建的网站上传至服务器，可能会导致网站无法正常显示。如果你使用的是Git，可以在`.gitignore`文件中添加`site`，这样就不会将`site`文件夹上传至服务器。

    ```bash
    # Ignore site directory
    site/
    ```

### 上传

依次执行以下命令。master如果不行，可以换成main。

```bash
git add .
git commit -m "update"
git push origin master  # 或者是 git push origin main
```
如果你想使用github pages托管你的网站，可以参考[GitHub Docs](https://docs.github.com/en/github/working-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)。运行以下命令，然后在github上选择master branch。(或者是main branch)

```bash
mkdocs gh-deploy # 这是mkdocs提供的一个命令，可以直接将网站部署到github pages上
```

!!! tip
    我本人使用一个脚本文件，一键上传并部署，非常方便。你可以参考我的脚本文件（upload.sh），然后根据自己的需求进行修改。在脚本所在的目录下，运行以下命令，然后就可以一键上传并部署了。

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

### 部署

在服务器上，首先需要拉取远程库。

如果是第一次拉取，实际上就是克隆远程库。

```bash
git clone <remote repository URL>
```

如果不是第一次拉取，就是拉取远程库的更新。

```bash
git pull
```

然后构建网站。

```bash
mkdocs build
```

最后挂载到nginx。 这一步参考第一种方法。

## 结语

至此，我们就完成了一个简洁、易于维护的技术博客的搭建。Enjoy it!