import rclpy
from rclpy.node import Node
from firebase_admin import credentials, db, initialize_app
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from rclpy.executors import MultiThreadedExecutor

"""
NOTE: This code is partially generated using ChatGPT
"""

class FirebaseListener(Node):
    def __init__(self):
        super().__init__('firebase_listener')

        # Initialize Firebase
        self.cred = credentials.Certificate('./firebase_credentials.json')
        self.db_name = ""
        initialize_app(self.cred, {
            'databaseURL': 'https://drone-control-db.firebaseio.com/'
        })

        self.area_id = 1

        # Timer to periodically check Firebase
        self.timer = self.create_timer(5.0, self.check_firebase)

        # Publisher for waypoints
        self.waypoint_pub = self.create_publisher(PoseArray, 'new_mission', 10)

    def check_firebase(self):
        ref = db.reference('Mission')
        missions = ref.get()

        waypoints_list = []

        for mission_id, mission_data in missions.items():
            if mission_data['hallway'] == self.area_id and not mission_data['completed']:
                waypoints = mission_data.get('waypoints', {})
                for wp_id, wp_data in waypoints.items():
                    pos = wp_data['pos']
                    pose = wp_data['initial_pose']
                    waypoint = ((pos['x'], pos['y'], pos['z']), (pose['x'], pose['y'], pose['z'], pose['w']))
                    waypoints_list.append(waypoint)
                
                self.publish_waypoints(waypoints_list)

                self.mission_completed(mission_id)
                break

    def publish_waypoints(self, waypoints):
        pose_array = PoseArray()
        pose_array.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        
        for position, orientation in waypoints:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
            pose_array.poses.append(pose)

        self.waypoint_pub.publish(pose_array)
        self.get_logger().info(f'Published waypoints: {waypoints}')

    def mission_completed(self, mission_id):
        ref = db.reference(f'Mission/{mission_id}')
        ref.update({'completed': True})
        self.get_logger().info(f'Marked inspection {mission_id} as completed')

def main(args=None):
    rclpy.init(args=args)
    node = FirebaseListener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
