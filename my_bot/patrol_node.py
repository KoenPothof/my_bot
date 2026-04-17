import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.action import ActionClient
import math

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._sub = self.create_subscription(Bool, '/start_patrol', self.start_callback, 10)
        self.get_logger().info('Patrol node klaar — wacht op /start_patrol topic')

    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info('Patrol gestart!')
            self.send_waypoints()

    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def send_waypoints(self):
        # Yaw waarden:
        # 0.0       = naar rechts (noord)
        # math.pi/2 = naar boven (west)
        # math.pi   = naar links (zuid)
        # -math.pi/2 = naar beneden (oost)

        waypoints = [
            # Start bij origin
            self.make_pose(0.0, 0.0, 0.0),

            # Kamer linksboven inrijden
            self.make_pose(8.0, -8.0, math.pi/2),


            self.make_pose(7.0, 7.0, math.pi),

            # Hoek rechtsboven, draai naar boven
            self.make_pose(-7.0, -7.0, math.pi/2),  
            
            # Terug naar origin
            self.make_pose(0.0, 0.0, 0.0),   
        ]

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints

        self._client.wait_for_server()
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Waypoints geaccepteerd!')
        else:
            self.get_logger().warn('Waypoints geweigerd!')

def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()