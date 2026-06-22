#!/usr/bin/env python3
"""Testnode voor de knipperlichten.

Doorloopt automatisch alle standen (links → rechts → gevaar → uit) zodat je op
de HMI kunt zien of de lampen reageren. Het bericht volgt dezelfde weg als in
bedrijf:

    deze node  →  /indicators  →  mqtt_hmi_bridge  →  MQTT
                                   ├─ robot/outputs/lamp_links
                                   └─ robot/outputs/lamp_rechts  →  HMI

Voorwaarde: de mqtt_hmi_bridge moet draaien (start die bv. via my_bot_launch.py,
of los met:  ros2 run my_bot mqtt_hmi_bridge.py).

Stoppen met Ctrl+C — de lampen gaan dan automatisch uit.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Hoe lang elke stand blijft staan (seconden)
STEP_DURATION = 3.0

# De teststanden in volgorde, met wat je op de lampen zou moeten zien
SEQUENCE = [
    ('links',  'LINKS  → lamp_links AAN,  lamp_rechts UIT'),
    ('uit',    'UIT    → beide lampen UIT'),
    ('rechts', 'RECHTS → lamp_links UIT,  lamp_rechts AAN'),
    ('uit',    'UIT    → beide lampen UIT'),
    ('gevaar', 'GEVAAR → beide lampen AAN'),
    ('uit',    'UIT    → beide lampen UIT'),
]


class IndicatorTestNode(Node):

    def __init__(self):
        super().__init__('indicator_test_node')
        self._indicator_pub = self.create_publisher(String, '/indicators', 10)
        self._index = 0

        self.get_logger().info(
            'IndicatorTestNode gestart — doorloopt de standen elke '
            f'{STEP_DURATION:.0f}s. Kijk op de HMI of de lampen reageren. '
            'Ctrl+C om te stoppen.')

        # Eerste stap pas na STEP_DURATION: geeft de publisher tijd om verbinding
        # te maken met de mqtt_hmi_bridge, anders gaat het eerste bericht verloren.
        self.create_timer(STEP_DURATION, self._step)

    def _step(self):
        state, uitleg = SEQUENCE[self._index % len(SEQUENCE)]
        msg = String()
        msg.data = state
        self._indicator_pub.publish(msg)
        self.get_logger().info(f'[{state}]  {uitleg}')
        self._index += 1

    def all_off(self):
        msg = String()
        msg.data = 'uit'
        self._indicator_pub.publish(msg)


def main():
    rclpy.init()
    node = IndicatorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Lampen netjes uit bij afsluiten
        node.all_off()
        rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
