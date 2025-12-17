# GitHub Pages Deployment Guide

Your Physical AI Native Book is now configured for deployment on GitHub Pages. Follow these steps to enable it:

## Step 1: Enable GitHub Pages

1. Go to your repository: https://github.com/YusraAnum/Physical-Ai-book
2. Click on the **"Settings"** tab
3. Scroll down to the **"Pages"** section in the left sidebar
4. Under **"Source"**, select:
   - **Branch**: `gh-pages`
   - **Folder**: `/ (root)`
5. Click **"Save"**

## Step 2: Wait for Initial Deployment

After pushing the configuration, GitHub Actions will:
1. Automatically build your Docusaurus site
2. Deploy the built files to the `gh-pages` branch
3. This process takes 2-5 minutes for the first build

## Step 3: Access Your Live Site

Once deployed, your site will be available at:
- https://yusraanum.github.io/Physical-Ai-book/

## GitHub Actions Workflow

The deployment is handled by the workflow in `.github/workflows/deploy.yml` which:
- Triggers on every push to the `main` branch
- Builds your Docusaurus site using `npm run build`
- Deploys the output to the `gh-pages` branch
- GitHub Pages serves from this branch

## Future Updates

- Any changes you make to the `main` branch will automatically trigger a new deployment
- The site will update within a few minutes after each push
- You can monitor deployment status in the "Actions" tab of your repository

## Troubleshooting

If your site doesn't appear after 10 minutes:
1. Check the "Actions" tab for any build failures
2. Verify that the Pages source is set to the `gh-pages` branch
3. Ensure the workflow completed successfully

Your Physical AI Native Book is now ready to be deployed on GitHub Pages!