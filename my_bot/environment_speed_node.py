#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from nav2_msgs.msg import SpeedLimit
import math


# Snelheidsinstelling — pas aan naar jouw situatie
SPEED_NORMAL        = 0.5   # m/s — normale rijsnelheid
SPEED_TURN          = 0.2   # m/s — snelheid bij scherpe bocht

# Bocht detectie (zelfde methode als indicator_node)
TURN_THRESHOLD_DEGREES = 45.0  # graden — minimale hoekverandering voor scherpe bocht
LOOKAHEAD_FRACTION     = 0.6   # eerste 60% van het pad bekijken


class EnvironmentSpeedNode(Node):

    def __init__(self):
        super().__init__('environment_speed_node')

        self._patrol_state  = 'idle'
        self._turn_active   = False

        self.create_subscription(String, '/patrol_state', self._on_patrol_state, 10)
        self.create_subscription(Path,   '/plan',         self._on_plan,         10)

        self._speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)

        self.get_logger().info('EnvironmentSpeedNode gestart')


    def _on_patrol_state(self, msg: String):
        self._patrol_state = msg.data

        # Bij stilstand of stop: normale snelheid herstellen
        if msg.data in ('idle', 'gestopt', 'voltooid', 'fout'):
            self._set_speed(SPEED_NORMAL)
            self._turn_active = False


    def _on_plan(self, msg: Path):
        if self._patrol_state != 'rijdend':
            return

        if len(msg.poses) < 3:
            return

        is_turn, _ = self._detect_turn(msg)

        if is_turn and not self._turn_active:
            self._turn_active = True
            self.get_logger().info(
                f'Scherpe bocht vooruit — snelheid verlaagd naar {SPEED_TURN} m/s')
            self._set_speed(SPEED_TURN)
        elif not is_turn and self._turn_active:
            self._turn_active = False
            self.get_logger().info(
                f'Pad is recht — snelheid hersteld naar {SPEED_NORMAL} m/s')
            self._set_speed(SPEED_NORMAL)


    def _detect_turn(self, path: Path):
        poses     = path.poses
        n         = len(poses)
        lookahead = max(3, int(n * LOOKAHEAD_FRACTION))
        poses     = poses[:lookahead]

        start  = poses[0].pose.position
        middle = poses[len(poses) // 2].pose.position
        end    = poses[-1].pose.position

        angle_1 = math.atan2(middle.y - start.y, middle.x - start.x)
        angle_2 = math.atan2(end.y - middle.y,   end.x - middle.x)

        delta_rad     = math.atan2(math.sin(angle_2 - angle_1), math.cos(angle_2 - angle_1))
        delta_degrees = abs(math.degrees(delta_rad))

        if delta_degrees > TURN_THRESHOLD_DEGREES:
            direction = 'links' if (angle_2 - angle_1) > 0 else 'rechts'
            return True, direction

        return False, None


    def _set_speed(self, speed: float):
        msg            = SpeedLimit()
        msg.percentage = False
        msg.speed_limit = speed
        self._speed_pub.publish(msg)


def main():
    rclpy.init()
    node = EnvironmentSpeedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
