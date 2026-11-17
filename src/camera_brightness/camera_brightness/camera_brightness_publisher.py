import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
import time
from rclpy.qos import qos_profile_sensor_data

class CameraBrightnessPublisher(Node):
    def __init__(self):
        super().__init__('camera_brightness_publisher')

        self.brightness = 0.0
        self.last_image_time = None
        self.image_timeout = 1.0  # Seconds without a valid image

        self.subscriber_ = self.create_subscription(
            Image,
            '/image',
            self.image_callback,10
            #qos_profile_sensor_data
        )

        timer_period = 0.5  # seconds                
        self.publisher_ = self.create_publisher(String, 'camera_brightness', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        no_recent_image = (
            self.last_image_time is None
            or time.monotonic() - self.last_image_time > self.image_timeout
        )

        if no_recent_image:
            brightness = 0.0
            self.get_logger().warning(
                'No valid image received within 1 second; publishing 0.'
            )
        else:
            brightness = self.brightness

        msg = String()
        msg.data = f'Current Brightness: {brightness:.2f}'
        self.publisher_.publish(msg)

    @staticmethod
    def calculate_brightness(image: Image) -> float:
        """Return mean brightness (0-255) using OpenCV."""

        # Convert the ROS image bytes into a NumPy image array.
        # image.step is the full row length in bytes, including padding.
        np_image = np.frombuffer(
            image.data,
            dtype=np.uint8,
        ).reshape(image.height, image.step)

        # Remove any padding at the end of each row.
        channels = {
            "mono8": 1,
            "8uc1": 1,
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
        }[image.encoding.lower()]

        np_image = np_image[:, : image.width * channels]
        np_image = np_image.reshape(image.height, image.width, channels)

        # Convert color images to grayscale.
        # OpenCV expects BGR order for 3-channel images.
        if image.encoding.lower() == "rgb8":
            np_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)

        if channels > 1:
            gray_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)
        else:
            gray_image = np_image

        # cv2.mean returns a tuple: [blue, green, red, alpha].
        # For grayscale, only the first value is used.
        return cv2.mean(gray_image)[0]

    def image_callback(self, msg: Image) -> None:
            try:
                self.brightness = self.calculate_brightness(msg)
                self.last_image_time = time.monotonic()

                self.get_logger().info(
                    f'Image recieving: {msg.width}x{msg.height}, '
                    f'brightness={self.brightness:.4f}'
                )

            except ValueError as error:
                self.get_logger().warning(f'Could not calculate image brightness: {error}')
                return

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraBrightnessPublisher()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()