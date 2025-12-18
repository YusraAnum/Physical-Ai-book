# Vercel Deployment Configuration

This project needs different configurations for GitHub Pages and Vercel deployments because they have different base URL requirements:

- **GitHub Pages**: Uses subdirectory format (e.g., `https://yusraanum.github.io/Physical-Ai-book/`) requiring `baseUrl: '/Physical-Ai-book/'`
- **Vercel**: Uses root domain format (e.g., `https://your-project.vercel.app/`) requiring `baseUrl: '/'`

## Solution: Environment-based Configuration

The recommended approach is to use environment variables to set the correct base URL:

1. Create a new file `book-physical-ai-humanoid-robotics/docusaurus.config.js` (JavaScript version) that can read environment variables
2. Set environment variable in Vercel dashboard: `DEPLOYMENT_PLATFORM=vercel`
3. For GitHub Pages, the default configuration remains the same

## Alternative Solution: Separate Deployments

If you want to deploy to both platforms simultaneously:

1. **For GitHub Pages**: Use the current configuration with `baseUrl: '/Physical-Ai-book/'`
2. **For Vercel**: Create a temporary branch with `baseUrl: '/'` and deploy from there

## Quick Fix for Vercel

To fix your current Vercel deployment:

1. Update the `baseUrl` in `docusaurus.config.ts` to `'/'` instead of `'/Physical-Ai-book/'`
2. Re-deploy to Vercel

## Recommended Vercel Environment Variables

In your Vercel project settings, add:
- `DEPLOYMENT_PLATFORM` = `vercel`
- This can be used in a dynamic configuration

## Current Configuration

Your current configuration has:
- `baseUrl: '/Physical-Ai-book/'` (for GitHub Pages)
- `url: 'https://YusraAnum.github.io'` (for GitHub Pages)

For Vercel deployment, you need:
- `baseUrl: '/'`
- `url: 'https://your-project-name.vercel.app'` (Vercel will auto-configure this)