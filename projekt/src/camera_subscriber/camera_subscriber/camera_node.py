#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.dir = 0
        self.img = np.zeros((512, 512, 3), np.uint8)
        self.msg = Twist()
        cv2.imshow("projekt", self.img)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, cv2.setMouseCallback('projekt', self.action))
    
    def action(self, event, x, y, flags, param):
        if(event == cv2.EVENT_LBUTTONDOWN):
            self.point = (x, y)
            self.img = np.zeros((512, 512, 3), np.uint8)
            cv2.rectangle(self.img, (x, y), (x + 40, y + 40), (255,0,0), 3)
            cv2.imshow("projekt", self.img)
            self.get_logger().info('y = "%d"' % y)
            if(y < 256):
                self.dir = 1
            else:
                self.dir = 2
            if(self.dir == 1):
                self.msg.linear.x = 0.0
                self.msg.linear.y = 1.0
                self.msg.linear.z = 0.0
                self.msg.angular.x = 0.0
                self.msg.angular.y = 0.0
                self.msg.angular.z = 0.0
            elif(self.dir == 2):
                self.msg.linear.x = 0.0
                self.msg.linear.y = -1.0
                self.msg.linear.z = 0.0
                self.msg.angular.x = 0.0
                self.msg.angular.y = 0.0
                self.msg.angular.z = 0.0
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
