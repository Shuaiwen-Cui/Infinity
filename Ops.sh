# This file is for site maintenance operations
# To be runned on linux-based server only

# Clean the site files
rm -r site

# Build the site
mkdocs build

# Deploy the site
mkdocs serve