#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import paho.mqtt.client as mqtt


# MQTT topics die de FT2J HMI publiceert
MQTT_TOPIC_START    = 'robot/start_patrol'    # payload "1" = starten
MQTT_TOPIC_STOP     = 'robot/stop_patrol'     # payload "1" = stoppen

# MQTT topics waarop de robot status terugstuurt naar de HMI
MQTT_TOPIC_STATE             = 'robot/patrol_state'
MQTT_TOPIC_INDICATOR_STATUS  = 'robot/indicator_status'
MQTT_TOPIC_BUZZER            = 'robot/outputs/buzzer'

# HMI-uitgangen voor de knipperlichten (net als de buzzer een output)
MQTT_TOPIC_INDICATOR_LEFT    = 'robot/outputs/lamp_links'   # linker knipperlicht
MQTT_TOPIC_INDICATOR_RIGHT   = 'robot/outputs/lamp_rechts'  # rechter knipperlicht


class MqttHmiBridge(Node):

    def __init__(self):
        super().__init__('mqtt_hmi_bridge')

        self.declare_parameter('broker_host', 'localhost')
        self.declare_parameter('broker_port', 1883)

        broker_host = self.get_parameter('broker_host').get_parameter_value().string_value
        broker_port = self.get_parameter('broker_port').get_parameter_value().integer_value

        # ROS2 publishers — naar de patrol en indicator nodes
        self._start_pub = self.create_publisher(Bool,   '/start_patrol', 10)
        self._stop_pub  = self.create_publisher(Bool,   '/stop_patrol',  10)

        # ROS2 subscribers — status terug naar de HMI via MQTT
        self.create_subscription(String, '/patrol_state',     self._on_patrol_state,     10)
        self.create_subscription(String, '/indicator_status', self._on_indicator_status, 10)
        self.create_subscription(Bool,   '/buzzer',           self._on_buzzer,           10)
        self.create_subscription(String, '/indicators',       self._on_indicators,       10)

        # MQTT client opzetten
        self._mqtt = mqtt.Client(client_id='ros2_bridge')
        self._mqtt.on_connect    = self._on_mqtt_connect
        self._mqtt.on_message    = self._on_mqtt_message
        self._mqtt.on_disconnect = self._on_mqtt_disconnect

        try:
            self._mqtt.connect(broker_host, broker_port, keepalive=60)
            self._mqtt.loop_start()
            self.get_logger().info(
                f'MQTT verbonden met {broker_host}:{broker_port}')
        except Exception as e:
            self.get_logger().error(f'MQTT verbinding mislukt: {e}')

    # ── MQTT callbacks ────────────────────────────────────────────────────────

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT verbinding geslaagd')
            client.subscribe(MQTT_TOPIC_START)
            client.subscribe(MQTT_TOPIC_STOP)
        else:
            self.get_logger().error(f'MQTT verbinding geweigerd — code {rc}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f'MQTT verbinding verbroken (rc={rc}) — herverbinden...')

    def _on_mqtt_message(self, client, userdata, msg):
        topic   = msg.topic
        payload = msg.payload.decode('utf-8').strip()

        if topic == MQTT_TOPIC_START and payload == '1':
            self.get_logger().info('HMI: start patrol ontvangen via MQTT')
            out      = Bool()
            out.data = True
            self._start_pub.publish(out)

        elif topic == MQTT_TOPIC_STOP and payload == '1':
            self.get_logger().info('HMI: stop patrol ontvangen via MQTT')
            out      = Bool()
            out.data = True
            self._stop_pub.publish(out)

    # ── ROS2 → MQTT: status doorsturen naar HMI ──────────────────────────────

    def _on_patrol_state(self, msg: String):
        self._mqtt.publish(MQTT_TOPIC_STATE, msg.data)

    def _on_indicator_status(self, msg: String):
        self._mqtt.publish(MQTT_TOPIC_INDICATOR_STATUS, msg.data)

    def _on_buzzer(self, msg: Bool):
        self._mqtt.publish(MQTT_TOPIC_BUZZER, '1' if msg.data else '0')

    def _on_indicators(self, msg: String):
        """Zet het knipperlicht-commando om naar de HMI-uitgangen Q01 (links)
        en Q02 (rechts). 'gevaar' = beide aan, 'uit' = beide uit."""
        state = msg.data
        left  = state in ('links', 'gevaar')
        right = state in ('rechts', 'gevaar')
        self._mqtt.publish(MQTT_TOPIC_INDICATOR_LEFT,  '1' if left  else '0')
        self._mqtt.publish(MQTT_TOPIC_INDICATOR_RIGHT, '1' if right else '0')

    def destroy_node(self):
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttHmiBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
