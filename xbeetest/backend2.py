from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS  # Import CORS
from digi.xbee.devices import XBeeDevice
import json

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})  # Enable CORS to allow requests from any origin
socketio = SocketIO(app, cors_allowed_origins="*")  # Allow WebSocket connections from any origin

# XBee setup
PORT = "/dev/cu.usbserial-AG0JYY5U"  # Your XBee port
BAUD_RATE = 115200
REMOTE_NODE_ID = "DRONE_PI"
device = None

def open_xbee_device():
    global device
    device = XBeeDevice(PORT, BAUD_RATE)
    device.open()
    xbee_network = device.get_network()
    remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
    if remote_device is None:
        print("Could not find the remote device")
        return None
    return remote_device

remote_device = open_xbee_device()

@socketio.on('gui_data')
def handle_joystick_input(data):
    # Handle the multiple joystick data points coming from the frontend
    print(f"Received data from joystick: {data}")

    try:
        # Parse the data and handle multiple inputs like rudder_angle, sail_angle, and throttle
        rudder_angle = data.get('rudder_angle', 0)
        sail_angle = data.get('sail_angle', 0)
        throttle = data.get('throttle', 0)
        
        # Prepare the payload as a JSON string to send over XBee
        payload = json.dumps({
            'rudder_angle': rudder_angle,
            'sail_angle': sail_angle,
            'throttle': throttle
        })

        # Send the payload to the remote XBee device
        if remote_device:
            device.send_data(remote_device, payload)
            print(f"Sent data to remote device: {payload}")
        else:
            print("Remote device not available")
    
    except Exception as e:
        print(f"Error handling data: {e}")

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=3336)