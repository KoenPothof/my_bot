#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import math


# Drempelwaarden — pas aan naar jouw situatie
TURN_THRESHOLD_DEGREES  = 45.0  # hoeveel graden afwijking = scherpe bocht (via /plan)
TURN_ROTATION_THRESHOLD = 0.20  # rad/s — knipperlicht AAN boven deze waarde
TURN_OFF_THRESHOLD      = 0.17  # rad/s — knipperlicht UIT onder deze waarde (hysteresis)
SPEED_THRESHOLD         = 0.05  # m/s — onder deze snelheid staat de robot stil
MIN_BLINKER_DURATION    = 2.0   # seconden — minimale brandtijd van de blinker
HAZARD_DELAY            = 1.2   # seconden stilstand voor gevaarslichten aangaan
LOOKAHEAD_FRACTION      = 0.6   # vooruitkijken: eerste 60% van het pad


class IndicatorNode(Node):

    def __init__(self):
        super().__init__('indicator_node')

        # Interne toestandsvariabelen
        self._patrol_state  = 'idle'
        self._detour_active = False
        self._direction     = 'uit'
        self._current_speed = 0.0
        self._hazard_on     = False
        self._still_since   = None  # tijdstip waarop robot stil ging staan
        self._activated_at  = None  # tijdstip waarop blinker aanging

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

        # Timer voor gevaarslichten wanneer robot stilstaat
        self.create_timer(2.0, self._blink_hazard)

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

        # Bijhouden vanaf wanneer de robot stilstaat
        if self._current_speed < SPEED_THRESHOLD:
            if self._still_since is None:
                self._still_since = self.get_clock().now()
        else:
            self._still_since = None
            if self._hazard_on:
                self._hazard_on = False
                indicator_msg      = String()
                indicator_msg.data = 'uit'
                self._indicator_pub.publish(indicator_msg)
                self.get_logger().info('Gevaarslichten UIT — robot rijdt weer')


    # ── Bocht detecteren via rijcommando ──
    def _on_cmd_vel(self, msg: Twist):
        """Detecteer bochten op basis van de rotatiesnelheid in /cmd_vel."""
        if self._patrol_state != 'rijdend':
            return

        rotation = msg.angular.z
        speed    = msg.linear.x

        self.get_logger().debug(f'cmd_vel: speed={speed:.2f} rotation={rotation:.2f}')

        if abs(speed) > SPEED_THRESHOLD and abs(rotation) > TURN_ROTATION_THRESHOLD:
            # Bocht — knipperlicht aan
            direction = 'links' if rotation > 0 else 'rechts'
            if not self._detour_active or self._direction != direction:
                self._activate(direction)
        elif self._detour_active and abs(rotation) < TURN_OFF_THRESHOLD:
            # Rotatie onder 0.17 — bocht voorbij, knipperlicht uit (na min. 2s)
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

        # Plan zet alleen AAN — uitzetten gebeurt via cmd_vel (bocht is écht voorbij)
        if is_detour:
            if not self._detour_active or self._direction != direction:
                self._activate(direction)


    # ── Hoekverandering berekenen langs het pad ──
    def _calculate_detour(self, path: Path):
        """
        Bepaal of het pad een significante bocht maakt en in welke richting.
        Vergelijkt de hoek van begin naar midden met midden naar eind.
        Geeft (bool is_detour, str direction) terug.
        """
        poses = path.poses
        n     = len(poses)

        # Kijk alleen naar het voorste deel van het pad (vroeg detecteren)
        lookahead = max(3, int(n * LOOKAHEAD_FRACTION))
        poses  = poses[:lookahead]
        start  = poses[0].pose.position
        middle = poses[len(poses) // 2].pose.position
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


    # ── Gevaarslichten knipperen wanneer robot stilstaat ──
    def _blink_hazard(self):
        actief_rijdend = self._patrol_state in ('rijdend', 'wachten')

        if self._still_since is None or self._detour_active or not actief_rijdend:
            return

        stil_seconden = (self.get_clock().now() - self._still_since).nanoseconds / 1e9
        if stil_seconden < HAZARD_DELAY:
            return

        self._hazard_on = not self._hazard_on
        status = 'AAN' if self._hazard_on else 'UIT'
        self.get_logger().info(f'Gevaarslichten {status} — robot staat stil')

        indicator_msg      = String()
        indicator_msg.data = 'gevaar' if self._hazard_on else 'uit'
        self._indicator_pub.publish(indicator_msg)


    # ── Buzzer en knipperlicht aanzetten ──
    def _activate(self, direction: str):
        self._detour_active = True
        self._direction     = direction
        self._activated_at  = self.get_clock().now()

        # Buzzer kort aan — timer zet hem na 0.3s weer uit
        buzzer_msg      = Bool()
        buzzer_msg.data = True
        self._buzzer_pub.publish(buzzer_msg)

        def _stop_buzzer():
            off_msg      = Bool()
            off_msg.data = False
            self._buzzer_pub.publish(off_msg)
            self.destroy_timer(buzzer_timer)

        buzzer_timer = self.create_timer(0.3, _stop_buzzer)

        indicator_msg      = String()
        indicator_msg.data = direction
        self._indicator_pub.publish(indicator_msg)

        self.get_logger().info(
            f'Knipperlicht {direction.upper()} AAN — robot maakt bocht naar {direction}')


    # ── Buzzer en knipperlicht uitzetten ──
    def _deactivate(self):
        # Wacht minimaal MIN_BLINKER_DURATION seconden voor uitzetten
        if self._activated_at is not None:
            aan_seconden = (self.get_clock().now() - self._activated_at).nanoseconds / 1e9
            if aan_seconden < MIN_BLINKER_DURATION:
                return

        self._detour_active = False
        self._direction     = 'uit'
        self._activated_at  = None

        buzzer_msg      = Bool()
        buzzer_msg.data = False
        self._buzzer_pub.publish(buzzer_msg)

        indicator_msg      = String()
        indicator_msg.data = 'uit'
        self._indicator_pub.publish(indicator_msg)

        self.get_logger().info('Knipperlicht UIT')


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