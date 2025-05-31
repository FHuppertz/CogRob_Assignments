
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from imitation_learning.imitation_model import train_gmm, gmr_predict 

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        
        self.subscription = self.create_subscription(
            Path,
            '/demonstration_trajectory',  
            self.demonstration_callback,
            10)
        
        
        self.publisher_ = self.create_publisher(Path, '/trajectory', 10)

        # List of poses of demonstartion trajectory (min 100)
        # uses them to run the rest and publishes to /trajectory

    def demonstration_callback(self, msg: Path):
        points = []
        for pose_stamped in msg.poses:
            
            t = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z
            points.append([t, x, y, z])
        
        data = np.array(points)
        

        t_min = data[:, 0].min()
        t_max = data[:, 0].max()
        data[:, 0] = (data[:, 0] - t_min) / (t_max - t_min + 1e-9)
        
        self.get_logger().info('Training GMM with demonstration data...')
        gmm = train_gmm(data, n_components=5)
        
        
        t_values = np.linspace(0, 1, 100)
        generated_traj = gmr_predict(gmm, t_values)
        
        
        self.publish_trajectory(generated_traj)


    def publish_trajectory(self, traj):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'
        
        for pt in traj:
            pose = PoseStamped()
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = float(pt[2])
            path_msg.poses.append(pose)
        
        self.publisher_.publish(path_msg)
        self.get_logger().info(f'Published generated trajectory with {len(traj)} points.')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
