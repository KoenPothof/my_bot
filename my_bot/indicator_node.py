import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import math


# Drempelwaarden — pas aan naar jouw situatie
TURN_THRESHOLD_DEGREES = 15.0   # hoeveel graden afwijking = omweg
SPEED_THRESHOLD        = 0.05   # m/s — onder deze snelheid staat de robot stil


class IndicatorNode(Node):

    def __init__(self):
        super().__init__('indicator_node')

        # Interne toestandsvariabelen
        self._patrol_state  = 'idle'
        self._detour_active = False
        self._direction     = 'uit'
        self._current_speed = 0.0

        # Subscribers aanmaken
        self.create_subscription(String,   '/patrol_state', self._on_patrol_state, 10)
        self.create_subscription(Path,     '/plan',         self._on_plan,         10)
        self.create_subscription(Odometry, '/odom',         self._on_odom,         10)
        self.create_subscription(Twist,    '/cmd_vel',      self._on_cmd_vel,      10)

        # Publishers aanmaken
        self._buzzer_pub    = self.create_publisher(Bool,   '/buzzer',           10)
        self._indicator_pub = self.create_publisher(String, '/indicators',       10)
        self._status_pub    = self.create_publisher(String, '/indicator_status', 10)

        # Timer voor het publiceren van de status
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info('IndicatorNode gestart — wacht op /patrol_state')


    # ── Patrol staat bijhouden ──
    def _on_patrol_state(self, msg: String):
        """Ontvang de staat van de PatrolNode via /patrol_state."""
        self._patrol_state = msg.data

        # Indicatoren uitzetten als de robot niet meer rijdt
        inactive_states = ('idle', 'voltooid', 'gestopt', 'planning')
        if self._patrol_state in inactive_states and self._detour_active:
            self.get_logger().info(
                f'Patrol staat: {self._patrol_state} — indicatoren uit')
            self._deactivate()


    # ── Huidige snelheid bijhouden via odometrie ──
    def _on_odom(self, msg: Odometry):
        """Bereken de werkelijke rijsnelheid van de robot."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed = math.sqrt(vx**2 + vy**2)


    # ── Bocht detecteren via rijcommando ──
    def _on_cmd_vel(self, msg: Twist):
        """Detecteer bochten op basis van de rotatiesnelheid in /cmd_vel."""
        if self._patrol_state != 'rijdend':
            return

        rotation = msg.angular.z
        speed    = msg.linear.x

        # Robot beweegt én draait tegelijk — dit is een bocht
        if abs(speed) > SPEED_THRESHOLD and abs(rotation) > 0.1:
            direction = 'links' if rotation > 0 else 'rechts'
            if not self._detour_active or self._direction != direction:
                self.get_logger().info(
                    f'Bocht gedetecteerd via /cmd_vel: {direction} '
                    f'(rotatie: {rotation:.2f} rad/s)')
                self._activate(direction)
        else:
            # Geen rotatie meer — bocht voorbij
            if self._detour_active:
                self.get_logger().info('Bocht voorbij — indicatoren uit')
                self._deactivate()


    # ── Omweg detecteren via gepland pad ──
    def _on_plan(self, msg: Path):
        """Detecteer omwegen op basis van het geplande pad van Nav2."""
        if self._patrol_state != 'rijdend':
            return

        if len(msg.poses) < 3:
            return

        # Bereken of het pad een bocht bevat
        is_detour, direction = self._calculate_detour(msg)

        if is_detour:
            if not self._detour_active or self._direction != direction:
                self.get_logger().info(
                    f'Omweg gedetecteerd via /plan: {direction}')
                self._activate(direction)
        else:
            if self._detour_active:
                self.get_logger().info('Pad is recht — indicatoren uit')
                self._deactivate()


    # ── Hoekverandering berekenen langs het pad ──
    def _calculate_detour(self, path: Path):
        """
        Bepaal of het pad een significante bocht maakt en in welke richting.
        Vergelijkt de hoek van begin naar midden met midden naar eind.
        Geeft (bool is_detour, str direction) terug.
        """
        poses = path.poses
        n     = len(poses)

        # Drie referentiepunten langs het pad
        start  = poses[0].pose.position
        middle = poses[n // 2].pose.position
        end    = poses[-1].pose.position

        # Hoek van start naar midden en van midden naar eind
        angle_1 = math.atan2(middle.y - start.y,  middle.x - start.x)
        angle_2 = math.atan2(end.y    - middle.y, end.x    - middle.x)

        # Hoekverandering berekenen en normaliseren naar -180 tot 180 graden
        delta_rad     = angle_2 - angle_1
        delta_rad     = math.atan2(math.sin(delta_rad), math.cos(delta_rad))
        delta_degrees = math.degrees(delta_rad)

        # Controleer of de bocht groot genoeg is om als omweg te tellen
        if abs(delta_degrees) > TURN_THRESHOLD_DEGREES:
            direction = 'links' if delta_degrees > 0 else 'rechts'
            return True, direction

        return False, 'uit'


    # ── Buzzer en knipperlicht aanzetten ──
    def _activate(self, direction: str):
        """Zet buzzer en knipperlicht aan in de opgegeven richting."""
        self._detour_active = True
        self._direction     = direction

        buzzer_msg      = Bool()
        buzzer_msg.data = True
        self._buzzer_pub.publish(buzzer_msg)

        indicator_msg      = String()
        indicator_msg.data = direction
        self._indicator_pub.publish(indicator_msg)

        self.get_logger().info(
            f'Buzzer AAN · Knipperlicht {direction.upper()} AAN')


    # ── Buzzer en knipperlicht uitzetten ──
    def _deactivate(self):
        """Zet buzzer en knipperlicht uit."""
        self._detour_active = False
        self._direction     = 'uit'

        buzzer_msg      = Bool()
        buzzer_msg.data = False
        self._buzzer_pub.publish(buzzer_msg)

        indicator_msg      = String()
        indicator_msg.data = 'uit'
        self._indicator_pub.publish(indicator_msg)

        self.get_logger().info('Buzzer UIT · Knipperlicht UIT')


    # ── Huidige status publiceren voor monitoring ──
    def _publish_status(self):
        """Publiceer de huidige staat voor monitoring via /indicator_status."""
        status_msg      = String()
        status_msg.data = (
            f'patrol:{self._patrol_state} | '
            f'omweg:{self._detour_active} | '
            f'richting:{self._direction} | '
            f'snelheid:{self._current_speed:.2f}m/s'
        )
        self._status_pub.publish(status_msg)


# Startpunt van de node
def main():
    rclpy.init()
    node = IndicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()