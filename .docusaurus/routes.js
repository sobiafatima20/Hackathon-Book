import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Hackathon-Book/',
    component: ComponentCreator('/Hackathon-Book/', '001'),
    routes: [
      {
        path: '/Hackathon-Book/',
        component: ComponentCreator('/Hackathon-Book/', 'cb9'),
        routes: [
          {
            path: '/Hackathon-Book/',
            component: ComponentCreator('/Hackathon-Book/', '38b'),
            routes: [
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/', 'acb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/isaac-setup',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/isaac-setup', 'd5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/labs/lab1-isaac-setup',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/labs/lab1-isaac-setup', 'e47'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/labs/lab2-synthetic-data',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/labs/lab2-synthetic-data', '540'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/labs/lab3-vslam',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/labs/lab3-vslam', '62c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/labs/lab4-nav2-planner',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/labs/lab4-nav2-planner', '026'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/labs/lab5-sim-to-real',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/labs/lab5-sim-to-real', '752'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/navigation-stack',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/navigation-stack', '2d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/perception-pipeline',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/perception-pipeline', '70b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/references',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/references', 'a49'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ai-robot-brain/vslam-implementation',
                component: ComponentCreator('/Hackathon-Book/modules/ai-robot-brain/vslam-implementation', '32d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/', '574'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/gazebo-setup',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/gazebo-setup', '46d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/labs/lab1-gazebo-setup',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/labs/lab1-gazebo-setup', '076'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/labs/lab2-sensor-config',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/labs/lab2-sensor-config', '2d8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/labs/lab3-ros2-sync',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/labs/lab3-ros2-sync', '8be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/labs/lab4-unity-vis',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/labs/lab4-unity-vis', '782'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/references',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/references', 'fa6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/ros2-sync',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/ros2-sync', 'e31'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/sensor-simulation',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/sensor-simulation', '72c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/digital-twin/unity-integration',
                component: ComponentCreator('/Hackathon-Book/modules/digital-twin/unity-integration', 'e36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/', '955'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/ai-integration',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/ai-integration', 'b1e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/architecture',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/architecture', 'f80'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/labs/lab1-workspace-setup',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/labs/lab1-workspace-setup', '7df'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/labs/lab2-communication',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/labs/lab2-communication', '25a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/labs/lab3-urdf-humanoid',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/labs/lab3-urdf-humanoid', '8c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/labs/lab4-ai-agent',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/labs/lab4-ai-agent', 'd9b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/nodes-topics',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/nodes-topics', 'b19'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/references',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/references', 'e51'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/ros2-nervous-system/urdf-modeling',
                component: ComponentCreator('/Hackathon-Book/modules/ros2-nervous-system/urdf-modeling', '67b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/', 'b6f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/action-planning',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/action-planning', '014'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/asr-integration',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/asr-integration', 'b8a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/labs/lab1-asr-setup',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/labs/lab1-asr-setup', 'ca2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/labs/lab2-llm-integration',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/labs/lab2-llm-integration', 'fe3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/labs/lab3-action-execution',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/labs/lab3-action-execution', 'eb5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/labs/lab4-vision-integration',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/labs/lab4-vision-integration', '552'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/labs/lab5-vla-integration',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/labs/lab5-vla-integration', 'e2c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/llm-ros-bridge',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/llm-ros-bridge', 'c73'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/multimodal-perception',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/multimodal-perception', '0e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/modules/vla-system/references',
                component: ComponentCreator('/Hackathon-Book/modules/vla-system/references', 'a7a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/quickstart',
                component: ComponentCreator('/Hackathon-Book/quickstart', '5b8'),
                exact: true
              },
              {
                path: '/Hackathon-Book/reference/citations',
                component: ComponentCreator('/Hackathon-Book/reference/citations', '0a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/reference/glossary',
                component: ComponentCreator('/Hackathon-Book/reference/glossary', '6c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon-Book/',
                component: ComponentCreator('/Hackathon-Book/', '277'),
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
