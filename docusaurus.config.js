// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to understand your code
// See https://docusaurus.io/docs/api/docusaurus-config

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Technical Book',
  tagline: 'A comprehensive educational resource for students, early-career engineers, and developers transitioning from digital AI to physical robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://sobiafatima20.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Hackathon-Book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'sobiafatima20', // Usually your GitHub org/user name.
  projectName: 'Hackathon-Book', // Usually your repo name.

  onBrokenLinks: 'throw',
  markdown: {
    mermaid: false,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true
    },
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    }
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Set the routeBasePath to '/' to serve docs from the root
          routeBasePath: '/',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/sobiafatima20/Hackathon-Book/edit/main/',
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Book Logo',
          src: 'img/logo.svg',
          href: '/Hackathon-Book/',  // Link logo to base URL
          target: '_self',
        },
        items: [
          {
            type: 'doc',
            docId: 'index',
            position: 'left',
            label: 'Home',
          },
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            href: 'https://github.com/sobiafatima20/Hackathon-Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
        hideOnScroll: false,
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'ROS 2 Nervous System',
                to: '/modules/ros2-nervous-system',
              },
              {
                label: 'Digital Twin',
                to: '/modules/digital-twin',
              },
              {
                label: 'AI-Robot Brain',
                to: '/modules/ai-robot-brain',
              },
              {
                label: 'Vision-Language-Action',
                to: '/modules/vla-system',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'ROS Answers',
                href: 'https://answers.ros.org/questions/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/sobiafatima20/Hackathon-Book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Educational Project. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;