# Repo Instruction

## Project Structure
```bash
├── README.md : project instructions and guide
├── upload.sh : upload the file to the server
├── mkdocs.yml : mkdocs configuration file
├── .editorconfig : editor configuration file
├── .gitignore : git ignore file. note that, better build the website in the server rather than locally
├── material : theme materials
└── docs : documentation files, this is the root directory of the website
    │── static: static files
    │   │── images: images
    │   └── videos: videos
    │── blog: blog posts
    │   │── index.md: blog index page
    │   └── posts: blog posts
    │── index.en.md: home page - in English
    │── index.zh.md: home page - in Chinese

```