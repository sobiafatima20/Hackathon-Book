# Book Implementation Files

This directory contains the implementation files and code examples for the Physical AI & Humanoid Robotics Technical Book. These files support the educational content in the book by providing practical examples and hands-on exercises.

## Directory Structure

```
book/
├── ros2-workspace/         # ROS 2 packages and examples
│   ├── src/               # Source packages
│   │   ├── humanoid_description/ # URDF models for humanoid robot
│   │   ├── ros2_nervous_system_examples/ # Basic ROS 2 examples
│   │   ├── ai_agent/      # AI agent implementations
│   │   └── vla_integration/ # VLA system integration
│   ├── launch/            # Launch files for demonstrations
│   └── config/            # Configuration files
├── gazebo-models/         # Gazebo simulation models and worlds
│   ├── worlds/            # Gazebo world files
│   ├── models/            # Custom robot models
│   └── plugins/           # Custom Gazebo plugins
├── unity-scenes/          # Unity visualization scenes
│   ├── humanoid/          # Humanoid robot models for Unity
│   ├── environments/      # Unity environments
│   └── ros2_bridge/       # Unity-ROS 2 bridge configuration
├── isaac-configs/         # NVIDIA Isaac configurations
│   ├── perception/        # Perception pipeline configs
│   ├── navigation/        # Navigation stack configs
│   └── synthetic_data/    # Synthetic data generation configs
└── vla-examples/          # Vision-Language-Action implementation examples
    ├── asr/               # ASR system implementations
    ├── llm/               # LLM integration examples
    └── planning/          # Action planning implementations
```

## ROS 2 Workspace

The `ros2-workspace/` directory contains a standard ROS 2 workspace with packages that demonstrate concepts covered in the book:

- `humanoid_description`: URDF models for the humanoid robot
- `ros2_nervous_system_examples`: Basic examples of ROS 2 communication patterns
- `ai_agent`: Examples of AI agents integrated with ROS 2
- `vla_integration`: Integration examples for Vision-Language-Action systems

## Building the Workspace

To build the ROS 2 workspace:

```bash
cd book/ros2-workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Usage Examples

After building, you can run examples like:

```bash
# Run the basic talker-listener example
ros2 run ros2_nervous_system_examples talker
# In another terminal:
ros2 run ros2_nervous_system_examples listener

# Run the AI agent
ros2 run ai_agent simple_ai_agent
```

## Simulation Files

The other directories contain configuration files for various simulation environments:

- `gazebo-models/`: Files for physics-based simulation in Gazebo
- `unity-scenes/`: Files for visualization in Unity (requires Unity installation)
- `isaac-configs/`: Files for NVIDIA Isaac Sim (requires Isaac Sim installation)
- `vla-examples/`: Implementation examples for the Vision-Language-Action module

## License

The code examples in this directory are licensed under the Apache 2.0 License, consistent with ROS 2 standards.