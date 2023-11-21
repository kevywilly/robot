import rclpy
import signal
import threading
from flask_cors import CORS
from flask import Flask, Response, request
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import robot.common.image_utils as image_utils

class Api(Node):
    def __init__(self):
        super().__init__('app')

        self.left_image_bytes: bytes = None
        self.right_image_bytes: bytes = None

        # publishers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', qos.qos_profile_services_default)

        # subscriptions
        self.create_subscription(Image, "/left/image_raw", self.left_image_callback, 10)
        self.create_subscription(Image, "/right/image_raw", self.right_image_callback, 10)

    def drive(self, msg: Twist):
        self.velocity_publisher.publish(msg)
        
    def left_image_callback(self, msg: Image):
        self.left_image_bytes = image_utils.sensor_image_to_jpeg_bytes(msg)

    def right_image_callback(self, msg: Image):
        self.right_image_bytes = image_utils.sensor_image_to_jpeg_bytes(msg)
    
    def get_jpeg(self, index: int = 0):
        return self.left_image_bytes if index == 0 else self.right_image_bytes
            

def ros2_thread(node: Api):
    print('entering ros2 thread')
    rclpy.spin(node)
    node.destroy_node()
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


rclpy.init(args=None)
app_node = Api()

app = Flask(__name__)
CORS(app)
cors = CORS(app, resource={
    r"/*": {
        "origins": "*"
    }
})

prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)


def _get_stream(index: int = 0):
    while True:
        # ret, buffer = cv2.imencode('.jpg', frame)
        try:
            yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + app_node.get_jpeg(index) + b'\r\n'
            )  # concat frame one by one and show result
        except Exception as ex:
            pass
    
@app.route('/api/stream/<index>')
def stream(index: str):
    return Response(_get_stream(int(index)), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/stream')
def stream_input(input: str):
    return Response(_get_stream(0), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.post('/api/drive')
def apply_drive():
    data = request.get_json()

    t = Twist()
    try:
        t.linear.x = float(data["x"])
        t.linear.y = float(data["y"])
        t.angular.z = float(data["z"])
    except:
        t = Twist()

    app_node.velocity_publisher.publish(t)
    
    return data

@app.post('/api/stop')
def stop():
    app_node.velocity_publisher.publish(Twist())
    return {"x": 0, "y": 0, "z": 0}


def main(args=None):
    threading.Thread(target=ros2_thread, args=[app_node]).start()
    app.run(host="0.0.0.0", debug=True, use_reloader = False)

if __name__=="__main__":
    main()