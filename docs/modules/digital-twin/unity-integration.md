# Unity Integration for Visualization

## Overview

Unity provides high-quality 3D visualization capabilities for robotics applications. This section covers integrating Unity with ROS 2 for real-time visualization of humanoid robots and their environments.

## Installing Unity Hub and Unity 2022.3 LTS

### System Requirements

- Ubuntu 22.04 LTS (Linux support may be limited; Windows/macOS recommended for development)
- Graphics card with DirectX 10 or OpenGL 3.3+ support
- At least 8GB RAM (16GB+ recommended)
- 20GB+ free disk space

### Installation Steps

```bash
# Option 1: Install via Snap (if available)
sudo snap install unity-hub

# Option 2: Download from Unity website
# Visit https://unity.com/download and follow installation instructions
```

### Unity ROS 2 Bridge Setup

Unity provides the Unity Robotics Hub which includes:
- Unity ROS TCP Connector
- Sample projects and tutorials
- Visualization tools for robotics

## Unity-ROS 2 Communication Architecture

### TCP/IP Bridge

The Unity ROS TCP Connector establishes communication between Unity and ROS 2:

```
ALGORITHM: Unity_ROS2_Bridge
INPUT: ros_topic_data
OUTPUT: unity_visualization

BEGIN
    ros_node = create_ros2_node("unity_bridge")
    unity_subscriber = create_subscriber(ros_node, "/robot_state", process_robot_state)

    FUNCTION process_robot_state(msg):
        unity_robot = get_unity_robot_object()
        unity_robot.position = msg.position
        unity_robot.rotation = msg.orientation
        unity_robot.update_visualization()
    END FUNCTION

    WHILE unity_running:
        process_ros_messages()
        update_unity_scene()
        yield_frame()
    END WHILE
END
```

## Setting Up Unity Scene for Humanoid Robot

### Basic Scene Structure

1. **Robot Model**: Import your humanoid robot model (URDF/SDF converted to Unity format)
2. **Environment**: Create or import environment models
3. **Camera System**: Set up cameras for visualization
4. **Lighting**: Configure lighting for realistic rendering
5. **ROS Bridge**: Implement ROS communication components

### Unity Robot Model Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    private string rosTopicName = "/robot_pose";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<PoseMsg>(rosTopicName, OnPoseReceived);
    }

    void OnPoseReceived(PoseMsg pose)
    {
        // Update robot position and rotation
        transform.position = new Vector3(
            (float)pose.position.x,
            (float)pose.position.y,
            (float)pose.position.z
        );

        transform.rotation = new Quaternion(
            (float)pose.orientation.x,
            (float)pose.orientation.y,
            (float)pose.orientation.z,
            (float)pose.orientation.w
        );
    }
}
```

## Unity ROS TCP Connector

### Installation

1. Open Unity Hub and create a new 3D project
2. In the Package Manager, install the "ROS TCP Connector" package
3. Add the ROSConnection prefab to your scene

### Basic Configuration

```csharp
using Unity.Robotics.ROSTCPConnector;

public class ROSConnectionManager : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize("127.0.0.1", 10000); // Default ROS bridge port
    }
}
```

## Visualization Techniques

### Real-time Robot State Visualization

1. **Joint Positions**: Visualize each joint's position and rotation
2. **Sensor Data**: Display sensor readings as overlays or indicators
3. **Trajectory**: Show planned and executed paths
4. **Environment Interaction**: Visualize collisions and interactions

### Multi-Camera Setup

```csharp
public class MultiCameraController : MonoBehaviour
{
    public Camera[] cameras;
    public int activeCameraIndex = 0;

    void Update()
    {
        // Switch between cameras
        if (Input.GetKeyDown(KeyCode.C))
        {
            cameras[activeCameraIndex].enabled = false;
            activeCameraIndex = (activeCameraIndex + 1) % cameras.Length;
            cameras[activeCameraIndex].enabled = true;
        }
    }
}
```

## Unity-ROS Bridge Configuration

### Launch File for Bridge

```xml
<!-- unity_bridge.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}]
        ),
        Node(
            package='unity_ros_websocket',
            executable='unity_ros_websocket',
            name='unity_ros_bridge',
            parameters=[
                {'server_ip': '127.0.0.1'},
                {'server_port': 5005}
            ]
        )
    ])
```

## Performance Considerations

### Rendering Optimization

1. **Level of Detail (LOD)**: Use different model complexities based on distance
2. **Occlusion Culling**: Don't render objects not visible to cameras
3. **Light Baking**: Pre-calculate static lighting to reduce runtime cost
4. **Texture Compression**: Use appropriate compression formats

### Network Optimization

1. **Message Throttling**: Limit message rate to reduce network load
2. **Data Compression**: Compress large data like point clouds
3. **Selective Publishing**: Only send necessary data to Unity

## Troubleshooting Unity-ROS Integration

### Common Issues

1. **Connection Problems**: Verify network settings and firewall configuration
2. **Message Format Issues**: Ensure ROS message types match Unity expectations
3. **Performance Issues**: Monitor frame rate and optimize accordingly
4. **Synchronization Problems**: Handle timing differences between Unity and ROS

### Debugging Tips

```csharp
// Enable debug logging
void OnPoseReceived(PoseMsg pose)
{
    Debug.Log($"Received pose: {pose.position.x}, {pose.position.y}, {pose.position.z}");
    // Update robot position
}
```

## Advanced Visualization Features

### Sensor Visualization

```csharp
public class LiDARVisualizer : MonoBehaviour
{
    private LineRenderer lineRenderer;
    public float[] scanData;

    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = scanData.Length;
    }

    void UpdateScanVisualization(float[] newScanData)
    {
        scanData = newScanData;
        Vector3[] positions = new Vector3[scanData.Length];

        for (int i = 0; i < scanData.Length; i++)
        {
            float angle = i * 0.1f; // Assuming 0.1 radian increment
            positions[i] = new Vector3(
                scanData[i] * Mathf.Cos(angle),
                0,
                scanData[i] * Mathf.Sin(angle)
            );
        }

        lineRenderer.SetPositions(positions);
    }
}
```

## Integration Best Practices

1. **Separation of Concerns**: Keep ROS communication separate from Unity rendering
2. **Error Handling**: Implement graceful degradation when connection is lost
3. **Data Validation**: Verify data integrity before visualization
4. **Modular Design**: Create reusable components for different robots

## Next Steps

Continue to [Sensor Simulation in Gazebo](./sensor-simulation.md) to learn about configuring various sensors in the simulation environment.