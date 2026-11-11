import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class YOLOObstacleDetector(Node):

    def __init__(self):
        super().__init__('yolo_obstacle_detector')

        self.bridge = CvBridge()

        # Weights default to best.pt next to this script; override with --ros-args -p weights:=/path/to/best.pt
        default_weights = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'best.pt')
        weights = self.declare_parameter('weights', default_weights).value
        self.get_logger().info(f'Loading YOLO weights from {weights}')
        self.model = YOLO(weights)

        # Keep only the newest frame: inference on the CPU is slower than the camera, and a deeper
        # queue would annotate stale frames (so the obstacle snapshot would show the trail before the stop).
        self.image_sub = self.create_subscription(Image, '/husky1/camera/image', self.image_callback,
                                                  qos_profile_sensor_data)

        self.detection_pub = self.create_publisher(String, '/husky1/obstacle_detections', 10)

        self.front_obstacle_pub = self.create_publisher(String, '/husky1/front_obstacle', 10)

        self.annotated_pub = self.create_publisher(Image, '/husky1/yolo_image', 10)

        self.get_logger().info('YOLO obstacle detector started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        results = self.model.predict(
            frame,
            imgsz=320,
            conf=0.25,
            iou=0.5,
            device='cpu',
            verbose=False
        )

        detections = []

        front_obstacle = None
        best_score = -1
        image_width = frame.shape[1]
        # Middle third of the image: a narrow rock a little off-centre (e.g. on a bend) still counts as in front
        centre_left = image_width / 3
        centre_right = 2 * image_width / 3

        for box in results[0].boxes:

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = self.model.names[class_id]

            detections.append(f'{class_name}: {confidence:.2f}')

            # Check whether the obstacle overlaps the middle third of the camera image
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            if x1 <= centre_right and x2 >= centre_left:
                box_area = (x2 - x1) * (y2 - y1)

                if box_area > best_score:
                    best_score = box_area
                    front_obstacle = class_name

        detection_msg = String()
        detection_msg.data = ', '.join(detections) if detections else 'None'

        self.detection_pub.publish(detection_msg)
        front_msg = String()
        front_msg.data = front_obstacle if front_obstacle else 'None'
        self.front_obstacle_pub.publish(front_msg)

        annotated_frame = results[0].plot()

        annotated_msg = Image()
        annotated_msg.height = annotated_frame.shape[0]
        annotated_msg.width = annotated_frame.shape[1]
        annotated_msg.encoding = 'bgr8'
        annotated_msg.is_bigendian = 0
        annotated_msg.step = annotated_frame.shape[1] * 3
        annotated_msg.data = annotated_frame.tobytes()

        annotated_msg.header = msg.header

        self.annotated_pub.publish(annotated_msg)


def main():
    rclpy.init()

    node = YOLOObstacleDetector()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Ctrl+C / SIGTERM may already have shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
