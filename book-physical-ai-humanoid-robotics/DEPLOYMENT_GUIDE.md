# Deployment Guide for Physical AI Native Book

This guide will help you deploy your Physical AI Native Book project on Vercel.

## Prerequisites

- A GitHub account with access to the repository
- A Vercel account (sign up at https://vercel.com if you don't have one)

## Deployment Steps

1. **Sign in to Vercel**
   - Go to https://vercel.com
   - Click "Continue with GitHub"
   - Authorize Vercel to access your GitHub account

2. **Create a New Project**
   - Click on "New Project" button
   - Find and select your repository: `YusraAnum/Physical-Ai-book`
   - Click "Import"

3. **Configure the Project**
   - Root Directory: `book-physical-ai-humanoid-robotics`
   - Framework: Should auto-detect as Docusaurus
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Environment Variables: None needed for this project

4. **Deploy**
   - Click "Deploy" button
   - Wait for the build process to complete (2-5 minutes)
   - Your site will be deployed with a unique URL

5. **Custom Domain (Optional)**
   - Go to your project dashboard
   - Click on "Settings" → "Domains"
   - Add your custom domain if desired

## Project Structure

The main project files are located in the `book-physical-ai-humanoid-robotics` directory, which contains:

- All documentation modules and chapters
- Docusaurus configuration (`docusaurus.config.ts`)
- Custom components and styling
- Blog content
- Static assets

## Configuration

The `vercel.json` file in the `book-physical-ai-humanoid-robotics` directory is already configured for optimal Docusaurus deployment:

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

## Troubleshooting

If you encounter any issues during deployment:

1. **Build errors**: Make sure all dependencies are properly configured in `package.json`
2. **Routing issues**: The catch-all route in `vercel.json` handles client-side routing
3. **Environment variables**: This project doesn't require any environment variables

## Post-Deployment

- Your site will automatically rebuild and redeploy on every push to the `main` branch
- You can access deployment logs and settings from your Vercel dashboard
- Performance metrics and analytics are available in the Vercel dashboard