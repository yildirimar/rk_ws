import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class NumericalIKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        
        # --- ROBOT PARAMETRELERİ (FK Node'dan alındı) ---
        self.joint_names = [
            'base_to_rotary',    # J1
            'rotary_to_lower',   # J2
            'lower_to_upper',    # J3
            'upper_to_support',  # J4
            'support_to_wrist'   # J5
        ]
        
        # Robotun tam geometrik zinciri (URDF verileri)
        self.chain_params = [
            # Joint 1
            {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0]},
            # Joint 2
            {'xyz': [0.06000, 0.26000, 0.64000], 'rpy': [-0.17453, -1.57080, 0.00000], 'axis': [0.0, 0.0, 1.0]},
            # Joint 3
            {'xyz': [0.44000, 0.48717, -0.05626], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0]},
            # Joint 4
            {'xyz': [-0.05000, 0.52078, 0.04106], 'rpy': [0.01066, 0.08661, 1.69343], 'axis': [0.99714, 0.0, 0.07554]},
            # Joint 5
            {'xyz': [0.16404, 0.00001, 0.12645], 'rpy': [1.57080, 1.48353, -0.12217], 'axis': [0.99714, 0.0, 0.07554]},
            # End Effector (Fixed)
            {'xyz': [0.0, 0.0, 0.05], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 0.0]} 
        ]

        # Mevcut eklem açılarını saklamak için (Başlangıç tahmini olarak kullanılır)
        self.current_joints = [0.0] * 5

        # --- İLETİŞİM ---
        self.create_subscription(Point, '/ik_target', self.ik_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.get_logger().info('Numerical IK Node Hazır. Gerçek robot geometrisi kullanılıyor.')

    def joint_state_callback(self, msg):
        """Mevcut robot konumunu takip et (IK için başlangıç noktası)"""
        try:
            temp_joints = []
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    temp_joints.append(msg.position[idx])
            
            if len(temp_joints) == 5:
                self.current_joints = temp_joints
        except Exception:
            pass

    def ik_callback(self, msg):
        target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"Hedef Alındı: {target_pos}")

        # Nümerik çözümleme başlat
        solution = self.solve_ik_numerical(target_pos, initial_guess=self.current_joints)
        
        if solution is not None:
            self.send_command(solution)
        else:
            self.get_logger().warn("IK Çözümü Bulunamadı (Menzil dışı veya sıkışma)")

    def forward_kinematics(self, joints):
        """Verilen eklem açıları için Uç Nokta (X,Y,Z) hesaplar"""
        T_total = np.eye(4)
        
        # 5 Hareketli Eklem + 1 Sabit Uç
        for i in range(len(self.chain_params)):
            params = self.chain_params[i]
            
            # Son parça (End Effector) sabit dönüşümdür, açısı yoktur (0.0)
            theta = joints[i] if i < 5 else 0.0
            
            T_joint = self.compute_transform(params, theta)
            T_total = np.dot(T_total, T_joint)
            
        return T_total[:3, 3] # Sadece X, Y, Z kısmını döndür

    def compute_transform(self, params, theta):
        # 1. Translation
        T_trans = np.eye(4)
        T_trans[:3, 3] = params['xyz']

        # 2. Static Rotation
        r_static = R.from_euler('xyz', params['rpy'])
        T_rot_static = np.eye(4)
        T_rot_static[:3, :3] = r_static.as_matrix()

        # 3. Dynamic Joint Rotation
        axis = np.array(params['axis'])
        if np.linalg.norm(axis) > 0:
            r_dynamic = R.from_rotvec(axis * theta)
            T_rot_dynamic = np.eye(4)
            T_rot_dynamic[:3, :3] = r_dynamic.as_matrix()
        else:
            T_rot_dynamic = np.eye(4)

        return T_trans @ T_rot_static @ T_rot_dynamic

    def solve_ik_numerical(self, target_pos, initial_guess):
        """Jacobian Transpose / Pseudo-Inverse Metodu ile İteratif Çözüm"""
        q = np.array(initial_guess)
        alpha = 0.2  # Öğrenme hızı (Step size)
        max_iters = 100
        tolerance = 0.01 # 1 cm hata payı
        
        for i in range(max_iters):
            # 1. Mevcut konum
            current_pos = self.forward_kinematics(q)
            
            # 2. Hata vektörü
            error = target_pos - current_pos
            if np.linalg.norm(error) < tolerance:
                return q.tolist() # Hedefe ulaştık
            
            # 3. Jacobian Hesapla (Sayısal Türev ile)
            J = self.compute_jacobian(q)
            
            # 4. Açıyı güncelle: q_new = q + alpha * J_transpose * error
            # Pseudo-inverse daha kararlıdır: np.linalg.pinv(J)
            delta_q = np.dot(np.linalg.pinv(J), error)
            q = q + alpha * delta_q
            
            # Basit eklem limitleri (Opsiyonel - ±Pi)
            q = np.clip(q, -3.14, 3.14)
            
        return None # Çözüm bulunamadı

    def compute_jacobian(self, q, epsilon=1e-6):
        """Sayısal Jacobian Matrisi (3x5)"""
        J = np.zeros((3, 5))
        current_pos = self.forward_kinematics(q)
        
        for i in range(5):
            q_perturbed = q.copy()
            q_perturbed[i] += epsilon
            pos_perturbed = self.forward_kinematics(q_perturbed)
            
            # Türev: (f(x+h) - f(x)) / h
            derivative = (pos_perturbed - current_pos) / epsilon
            J[:, i] = derivative
            
        return J

    def send_command(self, joints):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumericalIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
