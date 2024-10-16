import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time

class FakeBoatLocationPublisher(Node):
    def __init__(self):
        super().__init__('fake_boat_location_publisher')

        # Create a publisher on the 'boat_locations' topic
        self.publisher_ = self.create_publisher(String, 'boat_locations', 10)

        # Set a timer to publish data every second
        self.timer = self.create_timer(1.0, self.publish_location)

    def generate_random_location(self, lat_base, lon_base):
        """Generate random coordinates near a base latitude and longitude."""
        lat_offset = random.uniform(-0.01, 0.01)  # Random latitude offset
        lon_offset = random.uniform(-0.01, 0.01)  # Random longitude offset
        return round(lat_base + lat_offset, 6), round(lon_base + lon_offset, 6)

    def publish_location(self):
        """Generate and publish the fake boat locations."""
        # Generate random locations for two boats
        boat_data = {
            "boats": [
                {
                    "id": "boat_1",
                    "lat": self.generate_random_location(37.7749, -122.4194)[0],  # San Francisco
                    "lon": self.generate_random_location(37.7749, -122.4194)[1]
                },
                {
                    "id": "boat_2",
                    "lat": self.generate_random_location(34.0522, -118.2437)[0],  # Los Angeles
                    "lon": self.generate_random_location(34.0522, -118.2437)[1]
                }
            ],
            "timestamp": time.time()  # Current time in seconds since epoch
        }

        # Create a ROS2 String message
        msg = String()
        msg.data = json.dumps(boat_data)  # Convert the data to JSON format

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published data
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    node = FakeBoatLocationPublisher()  # Create the node

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass  # Gracefully handle Ctrl+C

    node.destroy_node()  # Destroy the node on exit
    rclpy.shutdown()  # Shutdown ROS2


if __name__ == '__main__':
    main()
