# Physical AI Native Book

This repository contains the Physical AI Native Book project - a comprehensive guide to ROS 2, humanoid robotics, Isaac Sim, and related topics.

## GitHub Pages Deployment

This project is configured for deployment on GitHub Pages using GitHub Actions.

### Automatic Deployment

1. The site automatically deploys when changes are pushed to the `main` branch
2. The workflow is defined in `.github/workflows/deploy.yml`
3. Built site is deployed to the `gh-pages` branch

### Accessing the Live Site

Once deployed, your site will be available at:
https://yusraanum.github.io/Physical-Ai-book/

### Manual Deployment Steps

If you need to enable GitHub Pages manually:

1. Go to your repository Settings
2. Navigate to "Pages" in the left sidebar
3. Under "Source", select "Deploy from a branch"
4. Select "gh-pages" branch and "/ (root)" folder
5. Click "Save"

### Site Configuration

- Base URL: `/Physical-Ai-book/` (for GitHub Pages subdirectory)
- Build directory: `book-physical-ai-humanoid-robotics/build`
- Framework: Docusaurus v3

### Local Development

To run the site locally:

```bash
cd book-physical-ai-humanoid-robotics
npm install
npm start
```

The site will be available at http://localhost:3000