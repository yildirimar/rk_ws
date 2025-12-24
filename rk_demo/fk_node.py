import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_names_ordered = [
            'base_to_rotary',    # Joint 1
            'rotary_to_lower',   # Joint 2
            'lower_to_upper',    # Joint 3
            'upper_to_support',  # Joint 4
            'support_to_wrist',  # Joint 5
            'wrist_to_end'       # Joint 6
        ]

        # -------------------------------------------------------------
        # EXACT URDF KINEMATIC CHAIN
        # We use the raw values from your XML file.
        # -------------------------------------------------------------
        self.chain_params = [
            # Joint 1: base_to_rotary
            {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0]},
            
            # Joint 2: rotary_to_lower
            {'xyz': [0.06000, 0.26000, 0.64000], 'rpy': [-0.17453, -1.57080, 0.00000], 'axis': [0.0, 0.0, 1.0]},
            
            # Joint 3: lower_to_upper
            {'xyz': [0.44000, 0.48717, -0.05626], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0]},
            
            # Joint 4: upper_to_support
            # If you "Cleaned" the URDF to be "1 0 0", change 'axis' below to [1.0, 0.0, 0.0].
            {'xyz': [-0.05000, 0.52078, 0.04106], 'rpy': [0.01066, 0.08661, 1.69343], 'axis': [0.99714, 0.0, 0.07554]},
            
            # Joint 5: support_to_wrist
            {'xyz': [0.16404, 0.00001, 0.12645], 'rpy': [1.57080, 1.48353, -0.12217], 'axis': [0.99714, 0.0, 0.07554]},

            # Joint 6: wrist_to_end
            {'xyz': [0.0, 0.0, 0.05], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0]}
        ]

        self.get_logger().info('FK Node has been started.')

    def listener_callback(self, msg):
        state_map = dict(zip(msg.name, msg.position))
        T_total = np.eye(4)

        # Loop through ALL joints using General Transforms
        for i, joint_name in enumerate(self.joint_names_ordered):
            params = self.chain_params[i]
            
            # Get theta (default to 0.0)
            theta = state_map.get(joint_name, 0.0)

            # Compute Exact Transform
            T_joint = self.compute_general_transform(
                params['xyz'], 
                params['rpy'], 
                params['axis'], 
                theta
            )
            
            
            T_total = np.dot(T_total, T_joint)

        self.broadcast_tf(T_total, msg.header.stamp)

        

    def compute_general_transform(self, xyz, rpy, axis_vector, theta):
        """
        Computes the matrix: Translate(xyz) * StaticRotate(rpy) * JointRotate(axis, theta)
        This works for ANY link geometry.
        """
        # 1. Translation
        T_trans = np.eye(4)
        T_trans[:3, 3] = xyz

        # 2. Static Rotation (URDF RPY)
        # We use 'xyz' order for Euler angles as per URDF standard
        r_static = R.from_euler('xyz', rpy)
        T_rot_static = np.eye(4)
        T_rot_static[:3, :3] = r_static.as_matrix()

        # 3. Dynamic Joint Rotation
        if np.linalg.norm(axis_vector) > 0:
            # Create a rotation vector: angle * unit_vector
            rot_vec = np.array(axis_vector) * theta
            r_dynamic = R.from_rotvec(rot_vec)
            T_rot_dynamic = np.eye(4)
            T_rot_dynamic[:3, :3] = r_dynamic.as_matrix()
        else:
            T_rot_dynamic = np.eye(4)

        # Combine: Parent -> Static Offset -> Static Rotation -> Joint Rotation
        return T_trans @ T_rot_static @ T_rot_dynamic

    def broadcast_tf(self, T, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'fk_end_effector'

        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]

        r = R.from_matrix(T[:3, :3])
        quat = r.as_quat()

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()