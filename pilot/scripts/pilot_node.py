#!/usr/bin/env python3
import rospy
import tf
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import CameraInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

# Your messages:
from yolo.msg import YoloDetections

# Import controllers
from navigation_controller import NavigationController
from grasping_controller import GraspingController



class PilotNode:
    def __init__(self):
        # Parameters
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.5)    # seconds
        self.detection_class = rospy.get_param("~detection_class", "burger")
        self.x_offset = rospy.get_param("~x_offset", 0.02)  # meters. for grasping forward/backward adjustment
        self.z_offset = rospy.get_param("~z_offset", -0.03)  # meters. for grasping height adjustment
        
        # Frame IDs
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        detection_topic = rospy.get_param("~detection_topic", "/detections")
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # State variables
        self.current_base_x = None  # Target position in base_link frame
        self.current_base_y = None
        self.current_base_z = None
        self.last_detection_time = None
        self.target_pose = None
        
        # State machine: "navigating", "grasping", "idle"
        self.state = "idle"

        # TF listener for coordinate transformations
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # Give tf listener time to fill buffer

        # Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.target_pose_pub = rospy.Publisher("target_pose", PointStamped, queue_size=1)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', 
                                         MultiRawIdPosDur, queue_size=1)
        
        # Subscriber
        self.det_sub = rospy.Subscriber(detection_topic, YoloDetections, self.detections_cb)
        
        # Initialize controllers
        nav_params = {
            'target_distance': rospy.get_param("~target_distance", 0.20),
            'distance_tolerance': rospy.get_param("~distance_tolerance", 0.05),
            'yaw_tolerance': rospy.get_param("~yaw_tolerance", 0.1),
            'max_linear_speed': rospy.get_param("~max_linear_speed", 0.05),
            'max_angular_speed': rospy.get_param("~max_angular_speed", 0.3),
            'Kp_distance': rospy.get_param("~Kp_distance", 1.0),
            'Kp_yaw': rospy.get_param("~Kp_yaw", 0.0005),
        }
        
        self.nav_controller = NavigationController(self.cmd_pub, nav_params)
        self.grasp_controller = GraspingController(self.joints_pub)
        
        # Initialize arm
        self.grasp_controller.initialize_arm()

        rospy.loginfo("[Pilot] Pilot node started.")
    def detections_cb(self, msg):
        """
        Look for the nearest target object and remember its position.
        Transform the position from camera frame to base_link frame.

        Message:
          yolo/YoloDetections
            std_msgs/Header header
            YoloDetection[] detections
        """
        # Don't update target during grasping
        if self.state == "grasping":
            return
            
        nearest_dist = None
        nearest_dx = None
        nearest_dy = None
        nearest_dz = None

        for det in msg.detections:
            # Filter for target class
            if det.class_name.lower() != self.detection_class.lower():
                continue

            curr_distance = det.distance_m
            # -1 means not available
            if curr_distance < 0:
                continue

            if nearest_dist is None or curr_distance < nearest_dist:
                nearest_dist = curr_distance
                nearest_dx = det.dx_m
                nearest_dy = det.dy_m
                nearest_dz = det.dz_m

        if nearest_dist is not None:
            # Transform target position from camera frame to base_link
            try:
                # Create a point in the camera frame
                point_camera = PointStamped()
                point_camera.header = msg.header
                point_camera.header.frame_id = self.camera_frame
                
                # Use dx, dy, dz from YOLO detection (already in meters)
                point_camera.point.x = nearest_dx
                point_camera.point.y = nearest_dy
                point_camera.point.z = nearest_dz
                
                # Transform to base_link frame
                point_base = self.tf_listener.transformPoint(self.base_frame, point_camera)
                
                # Apply offsets for grasping
                point_base.point.x += self.x_offset
                point_base.point.z += self.z_offset
                
                # Store and publish
                self.target_pose = point_base
                self.target_pose_pub.publish(self.target_pose)
                
                self.current_base_x = point_base.point.x
                self.current_base_y = point_base.point.y
                self.current_base_z = point_base.point.z
                self.last_detection_time = rospy.Time.now()
                
                # Update state to navigating if idle
                if self.state == "idle":
                    self.state = "navigating"
                
                rospy.loginfo("[Pilot] Target in base_link: x=%.3f, y=%.3f, z=%.3f", 
                             point_base.point.x, point_base.point.y, point_base.point.z)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("[Pilot] TF transform failed: %s", str(e))
                return
        else:
            # No target detected - only update last_detection_time if we were tracking something
            if self.state == "navigating" and self.last_detection_time is not None:
                # Target lost during navigation - let process_navigation handle timeout
                pass

    def process_navigation(self):
        """Process navigation to target."""
        # Check for detection timeout
        if self.last_detection_time is None:
            self.nav_controller.stop()
            self.state = "idle"
            return
            
        dt = (rospy.Time.now() - self.last_detection_time).to_sec()
        if dt > self.detection_timeout:
            rospy.loginfo("[Pilot] No target detected, stopping navigation")
            self.nav_controller.stop()
            self.state = "idle"
            # Clear target data
            self.current_base_x = None
            self.current_base_y = None
            self.current_base_z = None
            self.last_detection_time = None
            return

        if self.current_base_x is None or self.current_base_y is None:
            self.nav_controller.stop()
            self.state = "idle"
            return
        
        # Compute velocity command
        twist, reached_goal = self.nav_controller.compute_velocity_command(
            self.current_base_x, self.current_base_y
        )
        
        if reached_goal:
            # Goal reached - transition to grasping
            rospy.loginfo("[Pilot] Navigation complete! Starting grasp sequence...")
            self.nav_controller.stop()
            self.state = "grasping"
            self.execute_grasp_sequence()
        else:
            # Continue navigation
            self.nav_controller.publish_command(twist)
    
    def execute_grasp_sequence(self):
        """Execute the grasping sequence."""
        rospy.loginfo("[Pilot] Executing grasp sequence...")
        
        # Execute full pick and place
        success = self.grasp_controller.execute_full_sequence(
            self.current_base_x,
            self.current_base_y,
            self.current_base_z
        )
        
        if success:
            rospy.loginfo("[Pilot] Grasp sequence completed successfully!")
        else:
            rospy.logwarn("[Pilot] Grasp sequence failed!")
        
        # Reset state
        self.state = "idle"
        self.current_base_x = None
        self.current_base_y = None
        self.current_base_z = None
        self.last_detection_time = None

    def spin(self):
        """Main control loop."""
        rate_hz = rospy.get_param("~rate", 20.0)
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            if self.state == "navigating":
                self.process_navigation()
            elif self.state == "grasping":
                # Grasping is blocking, state will change when done
                pass
            elif self.state == "idle":
                # Waiting for detection
                pass
            
            rate.sleep()


def main():
    rospy.init_node("pilot_node")
    node = PilotNode()
    node.spin()


if __name__ == "__main__":
    main()
