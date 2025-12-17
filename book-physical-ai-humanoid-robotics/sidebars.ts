import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar structure for the ROS 2 book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: Fundamentals of ROS 2',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Introduction to ROS 2',
          items: [
            'module-1/chapter-1/middleware-concept',
            'module-1/chapter-1/ros2-role',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 2: Core Communication Primitives',
          items: [
            'module-1/chapter-2/nodes-topics-services',
            'module-1/chapter-2/message-passing',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 3: Advanced Communication Concepts',
          items: [
            'module-1/chapter-3/control-signals',
          ],
          collapsed: false,
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 in Practice',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Python-ROS Integration',
          items: [
            'module-2/chapter-1/python-ros-integration',
            'module-2/chapter-1/rclpy-examples',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 2: Robot Modeling and Control',
          items: [
            'module-2/chapter-2/urdf-introduction',
          ],
          collapsed: false,
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Isaac Sim Simulation Coverage',
          items: [
            'module-3/chapter-1/introduction-to-isaac-sim',
            'module-3/chapter-1/environment-capabilities',
            'module-3/chapter-1/synthetic-data-generation',
            'module-3/chapter-1/simulation-vs-real-world',
            'module-3/chapter-1/system-architecture',
            'module-3/chapter-1/comparison-approaches',
            'module-3/chapter-1/synthetic-data-pipeline',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 2: Isaac ROS Perception',
          items: [
            'module-3/chapter-2/introduction-to-isaac-ros',
            'module-3/chapter-2/hardware-accelerated-vslam',
            'module-3/chapter-2/sensor-fusion-techniques',
            'module-3/chapter-2/real-time-navigation-constraints',
            'module-3/chapter-2/sensor-fusion-architecture',
            'module-3/chapter-2/visual-diagrams',
            'module-3/chapter-2/chapter-summary',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 3: Nav2 Navigation for Humanoids',
          items: [
            'module-3/chapter-3/introduction-to-nav2',
            'module-3/chapter-3/path-planning-fundamentals',
            'module-3/chapter-3/adapting-nav2-bipedal',
            'module-3/chapter-3/obstacle-avoidance-strategies',
            'module-3/chapter-3/system-architecture',
            'module-3/chapter-3/visual-navigation-architecture',
            'module-3/chapter-3/chapter-summary-nav2',
          ],
          collapsed: false,
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Systems',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Voice-to-Action Pipelines',
          items: [
            'module-4/chapter-1/index',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 2: Cognitive Planning with LLMs',
          items: [
            'module-4/chapter-2/index',
          ],
          collapsed: false,
        },
        {
          type: 'category',
          label: 'Chapter 3: Capstone - Autonomous Humanoid',
          items: [
            'module-4/chapter-3/index',
          ],
          collapsed: false,
        },
      ],
      collapsed: false,
    },
    {
      type: 'doc',
      id: 'references',
      label: 'References'
    }
  ],
};

export default sidebars;
