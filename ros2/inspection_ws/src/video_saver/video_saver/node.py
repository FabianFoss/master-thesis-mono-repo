import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from firebase_admin import firestore, credentials, initialize_app
from deviation_detection.object_detection_service import find_deviations

"""
NOTE: This code is partially generated using ChatGPT-4 and ChatGPT-4o
"""

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            100)

        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_width = 640 
        self.frame_height = 480  
        self.fps = 30
        self.output_file = 'latest_inspection.mp4'
        self.recording = False
        self.last_frame_time = None
        self.timeout = 5.0  # seconds

        self.timer = self.create_timer(1.0, self.check_timeout)

        self.cred = credentials.Certificate('./firebase_credentials.json')
        self.firebase = initialize_app(self.cred)
        self.db = firestore.client()


    def listener_callback(self, msg):
        # self.get_logger().info('Received frame')
        self.last_frame_time = self.get_clock().now().to_msg().sec
        self.frame_width = msg.width
        self.frame_height = msg.height

        # Convert ROS2 Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.recording:
            self.start_recording(cv_image)

        if self.recording:
            self.video_writer.write(cv_image)

    def start_recording(self, frame):
        self.frame_height, self.frame_width, _ = frame.shape
        self.video_writer = cv2.VideoWriter(self.output_file, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (self.frame_width, self.frame_height))
        self.recording = True
        self.get_logger().info('Started recording')

    def stop_recording(self):
        if self.recording and self.video_writer is not None:
            self.video_writer.release()
            self.recording = False

        self.inspection_data = {
            "areaName": "A3",
            "buildingAreaId": 1,
            "date": self.get_clock().now().to_msg().sec,
            "droneId": 1,
            "errorMsg": "",
            "floorId": 3,
            "floorName": 5,
            "inspectionType": "escaperoute inspection",
            "status": "",
        }

        doc_ref = self.db.collection("Inspection").add(self.inspection_data)
        
        find_deviations(doc_ref.id, self.db, self.firebase)

        self.get_logger().info('Stopped recording')

    def check_timeout(self):
        if self.recording and self.last_frame_time is not None:
            current_time = self.get_clock().now().to_msg().sec
            if current_time - self.last_frame_time > self.timeout:
                self.stop_recording()

    # TODO: Create function that uploads inspection vide to database for post-processing funtionality
    def upload_inspection_to_database():
        return

    def destroy_node(self):
        self.stop_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    image_saver_node = ImageSaverNode()

    rclpy.spin(image_saver_node)

    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
