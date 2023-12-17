# Repo Instruction

## Project Structure
```bash
├── README.md : project instructions and guide
├── upload.sh : upload the file to the server
├── deploy.sh : clean the file, pull repo, build the website and upload to the server
├── ops.sh : to a series of operations on the server
├── mkdocs.yml : mkdocs configuration file
├── .editorconfig : editor configuration file
├── .gitignore : git ignore file. note that, better build the website in the server rather than locally
├── material : theme materials
├── site : built website to be deployed
└── docs : documentation files, this is the root directory of the website
    │── static: static files
    │   │── images: images
    │   └── videos: videos
    │── blog: blog posts
    │   │── index.md: blog index page
    │   └── posts: blog posts
    │── ReferenceCode: reference code for copy and paste
    │── index.md: home page - in English
    │── index.zh.md: home page - in Chinese
    └── Other pages

```