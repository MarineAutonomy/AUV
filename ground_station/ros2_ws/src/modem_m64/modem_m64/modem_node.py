import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from wlmodem import WlModem  # type: ignore

class ModemNode(Node):
    def __init__(self):
        super().__init__('acoustic_modem_driver')
        
        # 1. Declare Parameters (allows changing settings without editing code)
        self.declare_parameter('role', 'a')  # Default: Master
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('channel', 4)

        role = self.get_parameter('role').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().string_value
        channel = self.get_parameter('channel').get_parameter_value().integer_value

        # 2. Hardware Initialization
        self.get_logger().info(f"Connecting to modem on {port}...")
        self.modem = WlModem(port)
        
        if not self.modem.connect():
            self.get_logger().error(f"CRITICAL: Could not connect to Modem on {port}!")
            return

        # 3. Configure Role and Channel
        # Role 'a' = Master (PC), Role 'b' = Slave (AUV)
        if self.modem.cmd_configure(role, channel):
            self.get_logger().info(f"SUCCESS: Modem configured as Role '{role}' on Channel {channel}")
        else:
            self.get_logger().error("FAILED to configure modem roles.")

        # 4. ROS Communication Setup
        # Topic to PUBLISH data we hear from the water
        self.data_pub = self.create_publisher(String, 'modem/received_data', 10)
        
        # Topic to SUBSCRIBE to for data we want to send into the water
        self.send_sub = self.create_subscription(String, 'modem/send_command', self.send_callback, 10)
        
        # 5. Polling Timer (Checks hardware for new acoustic pings every 0.1s)
        self.timer = self.create_timer(0.1, self.poll_modem)

    def poll_modem(self):
        # Non-blocking check for incoming acoustic packets
        packet = self.modem.get_data_packet(timeout=0)
        if packet:
            try:
                decoded_msg = packet.decode('utf-8', errors='ignore')
                msg = String()
                msg.data = decoded_msg
                self.data_pub.publish(msg)
                self.get_logger().info(f"INCOMING ACOUSTIC: {decoded_msg}")
            except Exception as e:
                self.get_logger().warn(f"Failed to decode packet: {e}")

    def send_callback(self, msg):
        # Triggered when someone publishes to 'modem/send_command'
        self.get_logger().info(f"TRANSMITTING: {msg.data}")
        success = self.modem.cmd_queue_packet(msg.data.encode('utf-8'))
        if not success:
            self.get_logger().error("Failed to queue packet for transmission!")

def main(args=None):
    rclpy.init(args=args)
    node = ModemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
