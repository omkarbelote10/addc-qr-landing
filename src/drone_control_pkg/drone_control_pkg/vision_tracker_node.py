import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class VisionTracker(Node):

    def __init__(self):
        super().__init__('vision_tracker_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.target_pub = self.create_publisher(
            Point,
            '/target_pixel',
            10
        )

        # Color tracking range in HSV (Default: Red)
        self.lower_color = np.array([0, 120, 70])
        self.upper_color = np.array([10, 255, 255])

        self.get_logger().info('Vision tracker started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # show img
        cv2.imshow("camera preview", cv_image)
        cv2.waitKey(1)
        # Convert BGR to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create binary mask for the specified color
        mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)

        # Clean up mask noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in the binary mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Select the largest contour assuming it's the target
            largest_contour = max(contours, key=cv2.contourArea)

            # Ignore tiny noisy contours
            if cv2.contourArea(largest_contour) > 100:
                # Calculate image moments to find the centroid
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:
                    cx = float(M["m10"] / M["m00"])
                    cy = float(M["m01"] / M["m00"])

                    # Construct and publish the Point message
                    point_msg = Point()
                    point_msg.x = cx
                    point_msg.y = cy
                    point_msg.z = 0.0  # Unused for 2D pixel space

                    self.target_pub.publish(point_msg)
                    self.get_logger().debug(f'Target detected at pixel: x={cx:.1f}, y={cy:.1f}')


def main(args=None):
    rclpy.init(args=args)

    node = VisionTracker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()