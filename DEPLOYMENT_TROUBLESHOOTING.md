# Deployment Troubleshooting Guide

This guide will help you troubleshoot and resolve deployment issues for both GitHub Pages and Vercel.

## GitHub Pages Deployment Issues

### 1. Check GitHub Actions Status
- Go to your repository: https://github.com/YusraAnum/Physical-Ai-book
- Click on the "Actions" tab
- Look for any failed workflow runs
- If there are failures, check the logs for specific error messages

### 2. Verify GitHub Pages Settings
- Go to repository Settings → Pages
- Ensure Source is set to: `gh-pages` branch, `/ (root)` folder
- If you don't see the gh-pages branch, the GitHub Action may not have run yet

### 3. Trigger GitHub Action Manually
If the action hasn't run:
- Go to Actions tab in your repository
- Find the "Deploy to GitHub Pages" workflow
- Click "Run workflow" to trigger it manually
- Wait 2-5 minutes for completion

### 4. Common GitHub Pages Issues:
- **BaseUrl Issue**: Make sure baseUrl is `/Physical-Ai-book/` (with leading slash)
- **Branch Issue**: Ensure the `gh-pages` branch is created and GitHub Pages source points to it
- **Cache Issue**: Sometimes clearing Docusaurus cache helps: `npm run clear`

## Vercel Deployment Issues

### 1. Check Vercel Project Settings
- In your Vercel dashboard, verify the project settings:
  - Root Directory: `book-physical-ai-humanoid-robotics`
  - Build Command: `npm run build`
  - Output Directory: `build`

### 2. Verify vercel.json Configuration
- Make sure `book-physical-ai-humanoid-robotics/vercel.json` exists and contains:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "build"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ]
}
```

### 3. Common Vercel Issues:
- **Wrong directory**: Ensure Vercel is looking in the `book-physical-ai-humanoid-robotics` subdirectory
- **Build script**: Make sure `npm run build` works in the Vercel environment
- **Node version**: The project requires Node.js >= 20

## Testing Locally Before Deployment

To test if your project builds correctly:

```bash
cd book-physical-ai-humanoid-robotics
npm install
npm run build
npm run serve
```

Then visit http://localhost:3000 to see your site locally.

## Quick Fix Steps

1. **For GitHub Pages**:
   - Ensure GitHub Actions are enabled in your repository settings
   - Wait for the first successful workflow run to create the `gh-pages` branch
   - Verify Pages source settings

2. **For Vercel**:
   - Double-check that the root directory is set to `book-physical-ai-humanoid-robotics`
   - Verify that the `vercel.json` file exists in the correct location

## Getting Help

If you're still experiencing issues:

1. Check the specific error messages in GitHub Actions or Vercel logs
2. Share the error messages for more targeted troubleshooting
3. Verify that your repository is public if you want public access to the site

## Verification Commands

Run these commands locally to verify everything works:

```bash
# Check if the build works locally
cd book-physical-ai-humanoid-robotics
npm run build

# Verify the build directory was created
ls -la build/

# Serve locally to test
npm run serve
```