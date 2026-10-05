import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
 
class Counter(Node):
    def __init__(self):
        super().__init__('counter')
        self.create_subscription(PointCloud2, '/husky1/scan/points', self.cb_raw, 10)
        self.create_subscription(PointCloud2, '/husky1/scan/points_filtered', self.cb_filtered, 10)
    def cb_raw(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        print('RAW valid points:', len(pts))
    def cb_filtered(self, msg):
        pts = list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
        print('FILTERED valid points:', len(pts))
 
rclpy.init()
node = Counter()
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.5)
rclpy.shutdown()