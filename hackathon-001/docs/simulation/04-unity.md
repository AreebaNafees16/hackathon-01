# Chapter 4: Introduction to Unity for Robotics Simulation

Unity is a widely used game engine that can also be leveraged for robotics simulation.  
It allows creating realistic 3D environments, simulating physics, and integrating AI and robotics algorithms.

---

## 4.1 Why Use Unity for Robotics?

- **High-quality 3D Visualization**: Realistic rendering for environments and robots.  
- **Physics Simulation**: Built-in Rigidbody, colliders, and forces for accurate robot dynamics.  
- **Cross-platform**: Supports Windows, macOS, Linux, iOS, and Android.  
- **AI Integration**: Unity ML-Agents and ROS-TCP-Connector allow AI and robotics integration.

Unity is especially useful for humanoid robotics, navigation testing, and complex multi-agent simulations.

---

## 4.2 Unity Architecture for Robotics

1. **Scenes**: Each simulation environment is a Scene.  
2. **GameObjects**: Any object in the scene (robot, obstacles, sensors).  
3. **Components**: Add functionality to GameObjects (e.g., Rigidbody, Collider, Scripts).  
4. **Scripts**: Control behavior using C# scripts.  
5. **Physics Engine**: Unity calculates collisions, forces, and constraints automatically.

---

## 4.3 Setting up Unity for Robotics Simulation

1. **Install Unity Hub** and Unity Editor (preferably LTS version).  
2. **Install ROS-TCP-Connector** package from Unity Asset Store.  
3. **Create a new 3D project** in Unity.  
4. **Set up ROS 2 communication** using ROS-TCP-Connector:
   - Enables sending and receiving ROS messages between Unity and ROS 2 nodes.

---

## 4.4 Example: Controlling a Humanoid Robot

1. **Add a humanoid model** (URDF or custom robot) to the Unity scene.  
2. **Add Rigidbody and Colliders** for each movable part.  
3. **Attach a C# script** to control joints using ROS 2 topics:

```csharp
using UnityEngine;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

public class JointController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "joint_commands";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32Msg>(topicName, OnJointCommandReceived);
    }

    void OnJointCommandReceived(Float32Msg msg)
    {
        transform.localRotation = Quaternion.Euler(0, msg.data, 0);
    }
}
