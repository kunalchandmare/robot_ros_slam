import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraBrightnessPublisher(Node):
    def __init__(self):
        super().__init__('camera_brightness_publisher')

        self.subscriber_ = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.brightness = 0.0
                
        self.publisher_ = self.create_publisher(String, 'camera_brightness', 10)

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
            except ValueError as error:
                self.get_logger().warning(f'Could not calculate image brightness: {error}')
                return

            brightness_msg = String()
            brightness_msg.data = str(self.brightness)
            
            self.publisher_.publish(brightness_msg)