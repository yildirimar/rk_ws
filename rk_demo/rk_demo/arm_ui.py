#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PyQt5.QtCore import Qt

class RobotArmUI(QWidget):
    def __init__(self, node):
        super().__init__()
        
        # We use the node passed from main(), we do NOT create a new one or init rclpy here
        self.node = node
        
        # Create Publisher
        self.publisher = self.node.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # YOUR ROBOT JOINT NAMES
        self.joint_names = [
            'base_to_rotary',   # Joint 1
            'rotary_to_lower',  # Joint 2
            'lower_to_upper' ,   # Joint 3
            'upper_to_support',  # Joint 4
            'support_to_wrist',  # Joint 5
        ]
        
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("KUKA Robot Control Panel 🚀")
        self.setGeometry(100, 100, 400, 300)
        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []

        # UI Elements for 3 Joints
        joint_titles = ["Base (Rotate)", "Rotary", "Lower" , "Upper", "Support"]
        
        for i, name in enumerate(joint_titles):
            # Label
            lbl = QLabel(f"{name}: 0.00 rad")
            lbl.setAlignment(Qt.AlignCenter)
            layout.addWidget(lbl)
            self.labels.append(lbl)

            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314) # -3.14 rad
            slider.setMaximum(314)  # +3.14 rad
            slider.setValue(0)
            slider.valueChanged.connect(self.update_labels)
            layout.addWidget(slider)
            self.sliders.append(slider)

        # Send Button
        btn = QPushButton("MOVE (SEND)")
        btn.clicked.connect(self.send_command)
        btn.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 10px;")
        layout.addWidget(btn)
        
        # Reset Button
        btn_reset = QPushButton("RESET (HOME)")
        btn_reset.clicked.connect(self.reset_positions)
        layout.addWidget(btn_reset)

        self.setLayout(layout)

    def update_labels(self):
        for i, slider in enumerate(self.sliders):
            val = slider.value() / 100.0
            self.labels[i].setText(f"Joint {i+1}: {val:.2f} rad")

    def reset_positions(self):
        for slider in self.sliders:
            slider.setValue(0)
        self.send_command()

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(s.value() / 100.0) for s in self.sliders]
        point.time_from_start.sec = 2 
        
        msg.points.append(point)
        
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Command Sent: {point.positions}")

def main(args=None):
    # 1. Initialize ROS 2 (ONLY ONCE)
    rclpy.init(args=args)

    # 2. Initialize PyQt Application
    app = QApplication(sys.argv)

    # 3. Create the ROS Node
    node = Node('arm_ui')

    # 4. Create the GUI Window and pass the node to it
    window = RobotArmUI(node)
    window.show()

    # 5. Execution Loop
    try:
        sys.exit(app.exec_())
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()