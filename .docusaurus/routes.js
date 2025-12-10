import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-book/',
    component: ComponentCreator('/physical-ai-book/', '461'),
    routes: [
      {
        path: '/physical-ai-book/',
        component: ComponentCreator('/physical-ai-book/', 'fb1'),
        routes: [
          {
            path: '/physical-ai-book/',
            component: ComponentCreator('/physical-ai-book/', 'b53'),
            routes: [
              {
                path: '/physical-ai-book/modules/ai-robot-brain/',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/', 'bf7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/isaac-setup',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/isaac-setup', '602'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/labs/lab1-isaac-setup',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/labs/lab1-isaac-setup', 'acd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/labs/lab2-synthetic-data',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/labs/lab2-synthetic-data', 'da8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/labs/lab3-vslam',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/labs/lab3-vslam', '094'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/labs/lab4-nav2-planner',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/labs/lab4-nav2-planner', '746'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/labs/lab5-sim-to-real',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/labs/lab5-sim-to-real', '5c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/navigation-stack',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/navigation-stack', 'b59'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/perception-pipeline',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/perception-pipeline', 'c81'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/references',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/references', '9e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ai-robot-brain/vslam-implementation',
                component: ComponentCreator('/physical-ai-book/modules/ai-robot-brain/vslam-implementation', '469'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/', '07c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/gazebo-setup',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/gazebo-setup', '9ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/labs/lab1-gazebo-setup',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/labs/lab1-gazebo-setup', '8ee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/labs/lab2-sensor-config',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/labs/lab2-sensor-config', 'b5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/labs/lab3-ros2-sync',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/labs/lab3-ros2-sync', '5d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/labs/lab4-unity-vis',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/labs/lab4-unity-vis', 'd7c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/references',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/references', '1cd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/ros2-sync',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/ros2-sync', '033'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/sensor-simulation',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/sensor-simulation', 'd24'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/digital-twin/unity-integration',
                component: ComponentCreator('/physical-ai-book/modules/digital-twin/unity-integration', '108'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/', '08f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/ai-integration',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/ai-integration', '5db'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/architecture',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/architecture', 'f8a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/labs/lab1-workspace-setup',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/labs/lab1-workspace-setup', '988'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/labs/lab2-communication',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/labs/lab2-communication', 'ffd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/labs/lab3-urdf-humanoid',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/labs/lab3-urdf-humanoid', '6a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/labs/lab4-ai-agent',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/labs/lab4-ai-agent', 'd05'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/nodes-topics',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/nodes-topics', 'c89'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/references',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/references', '9c7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/ros2-nervous-system/urdf-modeling',
                component: ComponentCreator('/physical-ai-book/modules/ros2-nervous-system/urdf-modeling', '3f4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/', '8d3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/action-planning',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/action-planning', 'e1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/asr-integration',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/asr-integration', '03a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/labs/lab1-asr-setup',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/labs/lab1-asr-setup', 'f3c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/labs/lab2-llm-integration',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/labs/lab2-llm-integration', '7be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/labs/lab3-action-execution',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/labs/lab3-action-execution', 'a9e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/labs/lab4-vision-integration',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/labs/lab4-vision-integration', '917'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/labs/lab5-vla-integration',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/labs/lab5-vla-integration', '4cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/llm-ros-bridge',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/llm-ros-bridge', '2a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/multimodal-perception',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/multimodal-perception', 'c94'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/modules/vla-system/references',
                component: ComponentCreator('/physical-ai-book/modules/vla-system/references', '87a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/quickstart',
                component: ComponentCreator('/physical-ai-book/quickstart', '4ee'),
                exact: true
              },
              {
                path: '/physical-ai-book/reference/citations',
                component: ComponentCreator('/physical-ai-book/reference/citations', '98b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/reference/glossary',
                component: ComponentCreator('/physical-ai-book/reference/glossary', '626'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/',
                component: ComponentCreator('/physical-ai-book/', '66b'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
