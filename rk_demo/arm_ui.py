#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QFrame
from PyQt5.QtCore import Qt, QTimer

# TF (Transform) Dinlemek için gerekli kütüphaneler
from tf2_ros import Buffer, TransformListener
import rclpy.time

class RobotArmUI(QWidget):
    def __init__(self, node):
        super().__init__()
        
        # ROS Node'u alıyoruz
        self.node = node
        
        # Publisher (Robotu hareket ettirmek için)
        self.publisher = self.node.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # --- TF LISTENER (FK Node'dan gelen veriyi okumak için) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # --- ROS SPIN TIMER ---
        # GUI'nin donmaması için ROS işlemlerini (spin) bir Timer ile yapıyoruz.
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_node)
        self.ros_timer.start(20) # 20ms'de bir (50 Hz) ROS verilerini kontrol et

        # --- GUI UPDATE TIMER ---
        # Ekrandaki X,Y,Z yazısını güncellemek için ayrı bir timer
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_xyz_display)
        self.display_timer.start(100) # 100ms'de bir (10 Hz) ekranı yenile

        # Robot Eklem İsimleri
        self.joint_names = [
            'base_to_rotary',   # Joint 1
            'rotary_to_lower',  # Joint 2
            'lower_to_upper' ,  # Joint 3
            'upper_to_support', # Joint 4
            'support_to_wrist', # Joint 5
        ]
        
        self.init_ui()

    def spin_ros_node(self):
        """ROS callback'lerini (TF verisi vb.) işler"""
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def update_xyz_display(self):
        """FK Node tarafından yayınlanan konumu okur ve ekrana yazar"""
        try:
            # 'base_link' referansına göre 'fk_end_effector' konumunu soruyoruz
            # Time() parametresi '0' anlamına gelir, yani en son mevcut veriyi getirir.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                'fk_end_effector', 
                now
            )
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            self.xyz_label.setText(f"📍 End Effector Position\nX: {x:.3f} m\nY: {y:.3f} m\nZ: {z:.3f} m")
            self.xyz_label.setStyleSheet("background-color: #27ae60; color: white; font-size: 14px; font-weight: bold; padding: 10px; border-radius: 5px;")

        except Exception as e:
            # Henüz veri gelmediyse veya FK Node çalışmıyorsa
            self.xyz_label.setText("Waiting for FK Node...\n(Is fk_node running?)")
            self.xyz_label.setStyleSheet("background-color: #e74c3c; color: white; font-size: 14px; font-weight: bold; padding: 10px; border-radius: 5px;")

    def init_ui(self):
        self.setWindowTitle("KUKA Robot Control Panel 🚀")
        self.setGeometry(100, 100, 450, 500)
        layout = QVBoxLayout()

        # --- XYZ DISPLAY (YENİ EKLENEN KISIM) ---
        self.xyz_label = QLabel("Initializing...")
        self.xyz_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.xyz_label)
        
        # Ayırıcı Çizgi
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        self.sliders = []
        self.labels = []

        # UI Elements for Joints
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
        btn.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 10px; margin-top: 10px;")
        layout.addWidget(btn)
        
        # Reset Button
        btn_reset = QPushButton("RESET (HOME)")
        btn_reset.clicked.connect(self.reset_positions)
        btn_reset.setStyleSheet("padding: 5px;")
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
