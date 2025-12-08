#!/usr/bin/env python3
"""
USB Camera Publisher Node
Publishes images from the icspring USB camera to ROS topics.
"""
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np


class USBCameraPublisher:
    """Publishes USB camera images to ROS topics."""
    
    def __init__(self):
        rospy.init_node('usb_camera_publisher')
        
        # Parameters
        self.device_id = rospy.get_param("~device_id", 0)  # /dev/video0
        self.frame_id = rospy.get_param("~frame_id", "usb_camera_link")
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.fps = rospy.get_param("~fps", 30)
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)
        
        # Camera calibration parameters (optional, can be loaded from file)
        self.camera_name = rospy.get_param("~camera_name", "icspring_camera")
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = rospy.Publisher("~image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)
        
        # Open camera
        rospy.loginfo(f"[USBCameraPublisher] Opening camera device {self.device_id}")
        self.cap = cv2.VideoCapture(self.device_id)
        
        if not self.cap.isOpened():
            rospy.logerr(f"[USBCameraPublisher] Failed to open camera device {self.device_id}")
            rospy.signal_shutdown("Camera failed to open")
            return
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Get actual properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        rospy.loginfo(f"[USBCameraPublisher] Camera opened successfully")
        rospy.loginfo(f"[USBCameraPublisher] Resolution: {actual_width}x{actual_height}")
        rospy.loginfo(f"[USBCameraPublisher] FPS: {actual_fps}")
        
        # Build camera info message
        self.camera_info_msg = self.build_camera_info(actual_width, actual_height)
        
    def build_camera_info(self, width, height):
        """Build a basic camera info message."""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = width
        camera_info.height = height
        
        # Simple camera matrix (approximate values)
        # These should ideally come from camera calibration
        fx = fy = width  # Approximate focal length
        cx = width / 2.0
        cy = height / 2.0
        
        camera_info.K = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        
        camera_info.R = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        camera_info.P = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        camera_info.distortion_model = "plumb_bob"
        
        return camera_info
    
    def publish_frame(self):
        """Capture and publish a single frame."""
        ret, frame = self.cap.read()
        
        if not ret:
            rospy.logwarn("[USBCameraPublisher] Failed to capture frame")
            return False
        
        # Create timestamp
        timestamp = rospy.Time.now()
        
        # Convert to ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.frame_id
            
            # Publish image
            self.image_pub.publish(image_msg)
            
            # Publish camera info
            self.camera_info_msg.header.stamp = timestamp
            self.camera_info_pub.publish(self.camera_info_msg)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"[USBCameraPublisher] Error publishing frame: {e}")
            return False
    
    def run(self):
        """Main loop to continuously publish frames."""
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("[USBCameraPublisher] Starting to publish frames...")
        
        while not rospy.is_shutdown():
            self.publish_frame()
            rate.sleep()
    
    def shutdown(self):
        """Clean up resources."""
        rospy.loginfo("[USBCameraPublisher] Shutting down...")
        if self.cap is not None:
            self.cap.release()
        rospy.loginfo("[USBCameraPublisher] Camera released")


def main():
    try:
        publisher = USBCameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'publisher' in locals():
            publisher.shutdown()


if __name__ == '__main__':
    main()
