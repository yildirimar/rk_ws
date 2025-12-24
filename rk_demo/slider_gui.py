import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray # Required for Gazebo controllers
import tkinter as tk
from tkinter import ttk
import threading

class slider_node(Node):
    def __init__(self):
        super().__init__('slider_gui_node')
        
        # 1. PUBLISHERS
        # Topic A: For RViz and FK Node (Visualization)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Topic B: For Gazebo / Real Robot (Actuation)
        # We send an array of positions to the controller
        self.command_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        
        # 2. DEFINE JOINTS
        # These must be in the EXACT order your controller expects them!
        self.joint_names = [
            'base_to_rotary',
            'rotary_to_lower',
            'lower_to_upper',
            'upper_to_support',
            'support_to_wrist',
            
        ]
        
        # Store current values
        self.joint_values = {name: 0.0 for name in self.joint_names}

        self.get_logger().info("Slider GUI (Gazebo + RViz) Started")

    def publish_joints(self):
        current_time = self.get_clock().now().to_msg()

        # --- MSG 1: JointState (For RViz/FK) ---
        js_msg = JointState()
        js_msg.header.stamp = current_time
        js_msg.name = self.joint_names
        js_msg.position = [self.joint_values[name] for name in self.joint_names]
        self.joint_pub.publish(js_msg)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.joint_values[name] for name in self.joint_names]
        self.command_pub.publish(cmd_msg)


class App:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.title("Gazebo Robot Controller")
        self.root.geometry("400x550")

        self.sliders = {}
        
        title_label = ttk.Label(self.root, text="Robot Control", font=("Arial", 14, "bold"))
        title_label.pack(pady=10)

        # Create sliders
        for name in ros_node.joint_names:
            frame = ttk.Frame(self.root)
            frame.pack(fill='x', padx=20, pady=5)
            
            lbl = ttk.Label(frame, text=name, font=("Consolas", 10))
            lbl.pack(anchor='w')
            
            scale = tk.Scale(
                frame, 
                from_=-3.14, 
                to=3.14, 
                orient='horizontal', 
                resolution=0.01,
                command=lambda val, n=name: self.on_slider_change(n, val)
            )
            scale.set(0.0)
            scale.pack(fill='x')
            
            self.sliders[name] = scale

        # Reset Button
        btn_reset = ttk.Button(self.root, text="Reset Positions", command=self.reset_sliders)
        btn_reset.pack(pady=20)
        
        # Info Label
        info_lbl = ttk.Label(self.root, text="Publishing to:\n/joint_states\n/position_controller/commands", justify="center")
        info_lbl.pack(pady=5)

        # Start Update Loop
        self.root.after(50, self.update_ros)

    def on_slider_change(self, joint_name, value):
        self.ros_node.joint_values[joint_name] = float(value)

    def reset_sliders(self):
        for name, scale in self.sliders.items():
            scale.set(0.0)
            self.ros_node.joint_values[name] = 0.0

    def update_ros(self):
        # Allow ROS to process its backend
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        
        # Publish current values
        self.ros_node.publish_joints()
        
        # Schedule next update
        self.root.after(50, self.update_ros)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = slider_node()
    app = App(node)
    
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()