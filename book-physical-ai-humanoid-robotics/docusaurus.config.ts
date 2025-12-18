import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'ROS 2 as a Robotic Nervous System',
  tagline: 'Understanding ROS 2 for Humanoid Robotics Applications',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  //  IMPORTANT FOR VERCEL
  url: 'https://physical-ai-book-v53v-git-main-yusraanums-projects.vercel.app',
  baseUrl: '/',
  trailingSlash: false,

  // Not needed for Vercel (safe to keep, but unused)
  organizationName: 'YusraAnum',
  projectName: 'Physical-Ai-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.ts'),
          editUrl: 'https://github.com/YusraAnum/Physical-Ai-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/YusraAnum/Physical-Ai-book/tree/main/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } as Preset.Options,
    ],
  ],

  themeConfig: {
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
            { label: 'Introduction', to: '/docs/intro' },
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
      copyright: `Copyright © ${new Date().getFullYear()} ROS 2: Robotic Nervous System Book.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } as Preset.ThemeConfig,
};

export default config;
