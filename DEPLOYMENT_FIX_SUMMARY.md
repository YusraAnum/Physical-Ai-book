# Deployment Configuration Update

## Issue Identified
The Docusaurus site was not loading properly on Vercel because it was configured with GitHub Pages base URL (`/Physical-Ai-book/`) instead of root path (`/`).

## Solutions Implemented

### 1. Dynamic Configuration
- Created `docusaurus.config.js` that can read environment variables
- Added logic to detect deployment platform (Vercel vs GitHub Pages)
- Uses appropriate base URL based on deployment platform

### 2. Enhanced Package Scripts
- Added `build-vercel` script that sets proper base URL for Vercel
- Added `build-gh-pages` script for GitHub Pages deployment

### 3. Vercel-Specific Environment Variables
- Configured `vercel.json` to set `DEPLOYMENT_PLATFORM=vercel` and `BASE_URL=/`
- Ensures correct base URL when deployed on Vercel

## How It Works

For Vercel deployment:
- Environment variable `DEPLOYMENT_PLATFORM=vercel` is set
- The config detects this and uses `baseUrl: '/'`
- Site deploys to root path as expected on Vercel

For GitHub Pages deployment:
- Environment variable is not set or different
- The config defaults to `baseUrl: '/Physical-Ai-book/'`
- Site deploys to subdirectory as expected on GitHub Pages

## Deployment Commands

For local testing:
- GitHub Pages: `npm run build-gh-pages`
- Vercel: `npm run build-vercel`

## Files Updated
1. `docusaurus.config.js` - Dynamic configuration
2. `package.json` - Added deployment scripts
3. `vercel.json` - Added environment variables
4. Created `VERCEL_DEPLOYMENT_ISSUE.md` - Issue documentation