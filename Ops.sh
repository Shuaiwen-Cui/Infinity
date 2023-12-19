# This file is for site maintenance operations
# To be runned on linux-based server only

# Clean the site files
rm -r site

# Get the latest version of the site
git pull

# Build the site - and nginx / apache will serve it
mkdocs build

# Deploy the site
# mkdocs serve