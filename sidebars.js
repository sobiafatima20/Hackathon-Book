// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Technical Book',
      items: [
        'index',
        {
          type: 'category',
          label: 'Module 1: ROS 2 Robotic Nervous System',
          items: [
            'modules/ros2-nervous-system/index',
            'modules/ros2-nervous-system/architecture',
            'modules/ros2-nervous-system/nodes-topics',
            'modules/ros2-nervous-system/urdf-modeling',
            'modules/ros2-nervous-system/ai-integration',
            'modules/ros2-nervous-system/references',
            {
              type: 'category',
              label: 'Labs',
              items: [
                'modules/ros2-nervous-system/labs/lab1-workspace-setup',
                'modules/ros2-nervous-system/labs/lab2-communication',
                'modules/ros2-nervous-system/labs/lab3-urdf-humanoid',
                'modules/ros2-nervous-system/labs/lab4-ai-agent',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Digital Twin (Gazebo + Unity)',
          items: [
            'modules/digital-twin/index',
            'modules/digital-twin/gazebo-setup',
            'modules/digital-twin/unity-integration',
            'modules/digital-twin/sensor-simulation',
            'modules/digital-twin/ros2-sync',
            'modules/digital-twin/references',
            {
              type: 'category',
              label: 'Labs',
              items: [
                'modules/digital-twin/labs/lab1-gazebo-setup',
                'modules/digital-twin/labs/lab2-sensor-config',
                'modules/digital-twin/labs/lab3-ros2-sync',
                'modules/digital-twin/labs/lab4-unity-vis',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
          items: [
            'modules/ai-robot-brain/index',
            'modules/ai-robot-brain/isaac-setup',
            'modules/ai-robot-brain/perception-pipeline',
            'modules/ai-robot-brain/vslam-implementation',
            'modules/ai-robot-brain/navigation-stack',
            'modules/ai-robot-brain/references',
            {
              type: 'category',
              label: 'Labs',
              items: [
                'modules/ai-robot-brain/labs/lab1-isaac-setup',
                'modules/ai-robot-brain/labs/lab2-synthetic-data',
                'modules/ai-robot-brain/labs/lab3-vslam',
                'modules/ai-robot-brain/labs/lab4-nav2-planner',
                'modules/ai-robot-brain/labs/lab5-sim-to-real',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            'modules/vla-system/index',
            'modules/vla-system/asr-integration',
            'modules/vla-system/llm-ros-bridge',
            'modules/vla-system/action-planning',
            'modules/vla-system/multimodal-perception',
            'modules/vla-system/references',
            {
              type: 'category',
              label: 'Labs',
              items: [
                'modules/vla-system/labs/lab1-asr-setup',
                'modules/vla-system/labs/lab2-llm-integration',
                'modules/vla-system/labs/lab3-action-execution',
                'modules/vla-system/labs/lab4-vision-integration',
                'modules/vla-system/labs/lab5-vla-integration',
              ],
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/glossary',
        'reference/citations',
      ],
    },
  ],
};

module.exports = sidebars;