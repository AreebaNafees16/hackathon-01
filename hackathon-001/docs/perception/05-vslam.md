# Chapter 5: Introduction to VSLAM (Visual Simultaneous Localization and Mapping)

Visual SLAM (VSLAM) is a technique used by robots to **build a map of an unknown environment while simultaneously tracking their own position using visual data** from cameras.

---

## 5.1 What is VSLAM?

- **SLAM**: Simultaneous Localization and Mapping  
- **Visual SLAM**: Uses cameras (monocular, stereo, or RGB-D) as the primary sensor  
- Helps robots navigate unknown environments without GPS  

Key outputs of VSLAM:

- **Map**: A representation of the environment (2D or 3D)  
- **Pose**: The position and orientation of the robot relative to the map  

---

## 5.2 How VSLAM Works

1. **Feature Extraction**: Identify distinct points or key features in camera frames.  
2. **Feature Matching / Tracking**: Track these features across consecutive frames.  
3. **Pose Estimation**: Estimate robot motion (translation + rotation) using feature displacement.  
4. **Map Update**: Incrementally build a map based on observed features.  
5. **Loop Closure**: Detect revisited areas to correct drift and refine the map.

---

## 5.3 VSLAM Algorithms

Common VSLAM frameworks:

- **ORB-SLAM2 / ORB-SLAM3**: Popular open-source SLAM algorithms using ORB features.  
- **RTAB-Map**: RGB-D mapping for real-time 3D reconstruction.  
- **LSD-SLAM**: Direct SLAM using image intensities rather than features.  

---

## 5.4 ROS 2 Integration

1. **Install VSLAM package**  
   Example using ORB-SLAM3:

```bash
cd ~/ros2_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch VSLAM node
ros2 launch orb_slam3_ros orb_slam3_mono.launch.py

# Example Python Subscriber for VSLAM Pose
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VSLAMSubscriber(Node):
    def __init__(self):
        super().__init__('vslam_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/camera/pose',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Robot Pose: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VSLAMSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
