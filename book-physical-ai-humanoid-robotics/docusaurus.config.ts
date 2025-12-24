import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Determine base URL based on environment
const isVercel = process.env.DEPLOYMENT_PLATFORM === 'vercel' || process.env.VERCEL === '1';
const isLocalhost = !isVercel && !process.env.DEPLOYMENT_PLATFORM;
const baseUrl = process.env.BASE_URL || (isVercel ? '/' : isLocalhost ? '/' : '/Physical-Ai-book/');

// Determine URL based on deployment platform
const url = isVercel
  ? process.env.URL || 'https://your-project-name.vercel.app'
  : 'https://YusraAnum.github.io';

const config: Config = {
  title: 'Physical Ai humanoid Robotics',
  tagline: 'Understanding ROS 2 for Humanoid Robotics Applications',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: url,
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  // For Vercel or other platforms, it's typically '/'
  baseUrl: baseUrl,

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'YusraAnum', // Usually your GitHub org/user name.
  projectName: 'Physical-Ai-book', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/YusraAnum/Physical-Ai-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/YusraAnum/Physical-Ai-book/tree/main/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'ROS 2: Robotic Nervous System',
      logo: {
        alt: 'ROS 2 Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book Chapters',
        },
        {
          href: 'https://github.com/YusraAnum/Physical-Ai-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Content',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Chapter 1: ROS 2 as Middleware',
              to: '/docs/chapter-1/middleware-concept',
            },
            {
              label: 'Chapter 2: Communication Primitives',
              to: '/docs/chapter-2/nodes-topics-services',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/rolling/',
            },
            {
              label: 'Official Tutorials',
              href: 'https://docs.ros.org/en/rolling/Tutorials.html',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/YusraAnum/Physical-Ai-book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} ROS 2: Robotic Nervous System Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,

  plugins: [
    // Removed chat widget plugin as it's now integrated via Layout wrapper
  ],
  stylesheets: [],
  scripts: [
    {
      src: '/js/text-selection.js',
      async: true,
      defer: true,
    },
  ],
  headTags: [
    {
      tagName: 'script',
      attributes: {
        type: 'text/javascript',
      },
      innerHTML: `
        window.RAG_API_URL = '${
          process.env.REACT_APP_API_URL ||
          process.env.RAG_API_URL ||
          'http://localhost:8000'
        }';
      `,
    },
  ],
};

export default config;