# This file is for site maintenance operations
# To be runned on linux-based server only

# Clean the site files
rm -r site

# Build the site - and nginx / apache will serve it
mkdocs build

# Deploy the site
# mkdocs serve