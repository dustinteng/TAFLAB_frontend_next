import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json

class FakeBoatLocationPublisher(Node):
    def __init__(self):
        super().__init__('fake_boat_location_publisher')
        self.publisher_ = self.create_publisher(String, 'boat_locations', 10)
        self.timer = self.create_timer(1.0, self.publish_location)

    def publish_location(self):
        boat_data = {
            "id": "boat_1",
            "lat": random.uniform(-90, 90),
            "lon": random.uniform(-180, 180)
        }
        msg = String()
        msg.data = json.dumps(boat_data)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeBoatLocationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
