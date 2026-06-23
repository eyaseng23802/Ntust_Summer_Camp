import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Create publisher for the /joint_states topic
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer period set to 50 milliseconds (20 Hz)
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Define the exact joint names from your Robot802 assignment model
        self.joint_names = [
            'arm_0_joint', 
            'arm_1_joint', 
            'arm_2_joint', 
            'gripper_1_joint', 
            'gripper_2_joint'
        ]
        
        # Define 5 distinct target points (waypoints) in the joint space
        # Format: [arm_0, arm_1, arm_2, gripper_1, gripper_2]
        self.waypoints = [
            [0.0,   0.0,   0.0,   0.0,  0.0],   # Home position
            [1.57,  0.5,   0.8,   0.04, 0.04],  # Turn right, raise arm, open gripper
            [2.9,   -0.5,  1.5,   0.0,  0.0],   # Deep reach, close gripper
            [-1.57, 0.2,   -0.5,  0.04, 0.04],  # Swing left, drop arm, open gripper
            [0.0,   -0.2,  0.3,   0.0,  0.0]    # Returning home
        ]
        
        # Interpolation Tracking Parameters
        self.current_waypoint_idx = 0
        self.next_waypoint_idx = 1
        self.interpolation_steps = 60  # Number of 50ms cycles between waypoints (3 seconds per movement)
        self.current_step = 0
        
        # Set initial positions
        self.current_positions = list(self.waypoints[self.current_waypoint_idx])
        self.get_logger().info('Trajectory Generator Node initialized and running!')

    def timer_callback(self):
        # 1. Calculate interpolation fraction (0.0 to 1.0)
        self.current_step += 1
        alpha = self.current_step / self.interpolation_steps
        
        start_pt = self.waypoints[self.current_waypoint_idx]
        end_pt = self.waypoints[self.next_waypoint_idx]
        
        # 2. Smoothly interpolate each joint position
        for i in range(len(self.joint_names)):
            self.current_positions[i] = start_pt[i] + alpha * (end_pt[i] - start_pt[i])
            
        # 3. If target waypoint is reached, advance to the next index
        if self.current_step >= self.interpolation_steps:
            self.current_step = 0
            self.current_waypoint_idx = self.next_waypoint_idx
            self.next_waypoint_idx = (self.next_waypoint_idx + 1) % len(self.waypoints)
            
        # 4. Construct and publish the JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
