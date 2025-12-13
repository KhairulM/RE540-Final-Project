import rospy
import yaml
import os
import numpy as np
from sensor_msgs.msg import Image
from pilot.srv import IdentifyStore, IdentifyStoreRequest
from cv_bridge import CvBridge

from controllers import NavigationController

class ExploringHandler:
    def __init__(self, pilot_node):
        self.pilot_node = pilot_node
        rospy.loginfo("[ExploringHandler] Initialized exploring state handler.")
        
        # Read parameters
        self.store_locations = rospy.get_param("~store_locations", [])
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.max_depth_distance = rospy.get_param("~max_depth_distance", 3.0)
        self.depth_check_size = rospy.get_param("~depth_check_size", 10)
        
        # Get semantic map file path
        package_path = os.path.dirname(os.path.abspath(__file__))
        self.semantic_map_path = os.path.abspath(os.path.join(package_path, "..", "config", "semantic_map.yaml"))
        
        # Initialize semantic map data structure
        self.semantic_map = self.load_semantic_map()
        
        # Initialize navigation controller
        nav_params = {}
        self.nav_controller = NavigationController(self.pilot_node.cmd_pub, nav_params)
        self.nav_timeout = self.pilot_node.navigation_timeout
        
        # Initialize identify_store service client
        rospy.loginfo("[ExploringHandler] Waiting for identify_store service...")
        rospy.wait_for_service('/identify_store')
        self.identify_store_service = rospy.ServiceProxy('/identify_store', IdentifyStore)
        rospy.loginfo("[ExploringHandler] identify_store service connected")
        
        # Camera and depth image storage
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.camera_sub = rospy.Subscriber(self.camera_topic, Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        
        rospy.loginfo("[ExploringHandler] Found %d store locations to explore", len(self.store_locations))
        
    def camera_callback(self, msg):
        """Store the latest camera image."""
        self.latest_image = msg
    
    def depth_callback(self, msg):
        """Store the latest depth image."""
        self.latest_depth = msg
    
    def is_facing_store(self):
        """
        Check if robot is facing a store by analyzing center depth values.
        
        Returns:
            bool: True if center depth is within threshold (facing something close)
        """
        if self.latest_depth is None:
            rospy.logwarn("[ExploringHandler] No depth image available")
            return False
        
        try:
            # Convert ROS depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
            
            # Get image dimensions
            height, width = depth_image.shape
            center_y, center_x = height // 2, width // 2
            
            # Define region of interest (ROI) around center
            half_size = self.depth_check_size // 2
            roi_y1 = max(0, center_y - half_size)
            roi_y2 = min(height, center_y + half_size)
            roi_x1 = max(0, center_x - half_size)
            roi_x2 = min(width, center_x + half_size)
            
            # Extract center region
            center_region = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
            
            # Filter out invalid depth values (NaN, Inf)
            valid_depths = center_region[(center_region < 10.0) & np.isfinite(center_region)]
            
            if len(valid_depths) == 0:
                rospy.logwarn("[ExploringHandler] No valid depth readings in center region")
                return False
            
            # Calculate average depth
            avg_depth = np.mean(valid_depths)
            
            rospy.loginfo("[ExploringHandler] Average center depth: %.2f m (threshold: %.2f m)", 
                         avg_depth, self.max_depth_distance)
            
            # Check if depth is within threshold
            if avg_depth <= self.max_depth_distance:
                rospy.loginfo("[ExploringHandler] Robot is facing a store (depth: %.2f m)", avg_depth)
                return True
            else:
                rospy.loginfo("[ExploringHandler] No store in front (depth: %.2f m > %.2f m)", 
                            avg_depth, self.max_depth_distance)
                return False
                
        except Exception as e:
            rospy.logerr("[ExploringHandler] Error processing depth image: %s", str(e))
            return False
    
    def load_semantic_map(self):
        """Load the semantic map from yaml file."""
        if os.path.exists(self.semantic_map_path):
            try:
                with open(self.semantic_map_path, 'r') as f:
                    semantic_map = yaml.safe_load(f)
                    if semantic_map is None:
                        semantic_map = {}
                    rospy.loginfo("[ExploringHandler] Loaded semantic map from %s", self.semantic_map_path)
                    return semantic_map
            except Exception as e:
                rospy.logwarn("[ExploringHandler] Failed to load semantic map: %s", str(e))
                return {}
        else:
            rospy.loginfo("[ExploringHandler] Semantic map file not found, will create new one")
            return {}
    
    def save_semantic_map(self):
        """Save the semantic map to yaml file."""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.semantic_map_path), exist_ok=True)
            
            with open(self.semantic_map_path, 'w') as f:
                yaml.dump(self.semantic_map, f, default_flow_style=False)
            rospy.loginfo("[ExploringHandler] Saved semantic map to %s", self.semantic_map_path)
            return True
        except Exception as e:
            rospy.logerr("[ExploringHandler] Failed to save semantic map: %s", str(e))
            return False
    
    def update_store_in_map(self, store_id, location, store_type):
        """Update a store entry in the semantic map."""
        self.semantic_map[store_id] = {
            'location': location,
            'type': store_type
        }
        rospy.loginfo("[ExploringHandler] Updated %s: location=%s, type=%s", 
                     store_id, location, store_type)
    
    def navigate_to_location(self, x, y, yaw=0.0):
        """
        Navigate to a specific location using the navigation controller with move_base.
        
        Args:
            x, y: Target coordinates in map frame
            yaw: Target orientation in radians in map frame
            
        Returns:
            bool: True if navigation succeeded, False otherwise
        """
        rospy.loginfo("[ExploringHandler] Navigating to location (%.2f, %.2f)", x, y)
        
        # Use the navigation controller's move_base goal function
        success = self.nav_controller.navigate_to_goal(x, y, yaw, frame_id="map", timeout=self.nav_timeout)
        
        if success:
            rospy.loginfo("[ExploringHandler] Successfully reached location (%.2f, %.2f)", x, y)
        else:
            rospy.logwarn("[ExploringHandler] Failed to reach location (%.2f, %.2f)", x, y)
        
        return success
    
    def identify_current_store(self):
        """
        Call the identify_store service with the current camera image.
        
        Returns:
            tuple: (success, store_type, confidence, reasoning)
        """
        if self.latest_image is None:
            rospy.logwarn("[ExploringHandler] No camera image available")
            return False, "", "", "No camera image available"
        
        try:
            rospy.loginfo("[ExploringHandler] Calling identify_store service...")
            response = self.identify_store_service(self.latest_image)
            
            if response.success:
                rospy.loginfo("[ExploringHandler] Store identified as: %s (confidence: %s)", 
                            response.store_type, response.confidence)
                rospy.loginfo("[ExploringHandler] Reasoning: %s", response.reasoning)
            else:
                rospy.logwarn("[ExploringHandler] Store identification failed: %s", response.reasoning)
            
            return response.success, response.store_type, response.confidence, response.reasoning
            
        except rospy.ServiceException as e:
            rospy.logerr("[ExploringHandler] Service call failed: %s", str(e))
            return False, "", "", str(e)
    
    def run(self):
        """
        Main exploration loop: visit each store location and identify the store type.
        """
        rospy.loginfo("[ExploringHandler] Starting exploration of %d stores", len(self.store_locations))
        
        for idx, location in enumerate(self.store_locations):
            if rospy.is_shutdown():
                rospy.loginfo("[ExploringHandler] ROS shutdown requested, stopping exploration")
                break
            
            x, y = location[0], location[1]
            rospy.loginfo("[ExploringHandler] === Exploring store %d/%d at (%.2f, %.2f) ===", 
                         idx + 1, len(self.store_locations), x, y)
            
            # Navigate to the store location
            nav_success = False
            while not nav_success and not rospy.is_shutdown():
                nav_success = self.navigate_to_location(x, y)
                
                if not nav_success:
                    rospy.logwarn("[ExploringHandler] Retrying store identification due to navigation failure")
            
            # Try 3 different orientations to identify the store so that the robot can get a good view
            orientations = [0.0, 1.57, 3.14]
            for orientation in orientations:
                rospy.loginfo("[ExploringHandler] Adjusting orientation to %.2f radians for store identification", orientation)
                orientation_success = self.navigate_to_location(x, y, yaw=orientation)
                
                if not orientation_success:
                    rospy.logwarn("[ExploringHandler] Failed to correct orientation to %.2f radians", orientation)
                    continue
                
                # Check if robot is actually facing a store
                if not self.is_facing_store():
                    rospy.logwarn("[ExploringHandler] Robot is not facing a store at orientation %.2f radians, trying next orientation", orientation)
                    continue
                
                # Wait a moment for the robot to stabilize and get a clear image
                rospy.sleep(1.0)
                # Identify the store
                success, store_type, confidence, reasoning = self.identify_current_store()
                
                if not success:
                    rospy.logwarn("[ExploringHandler] Store identification failed at orientation %.2f radians", orientation)
                    continue
                
                if store_type == IdentifyStoreRequest.UNKNOWN.lower():
                    rospy.loginfo("[ExploringHandler] Store type identified as UNKNOWN, trying next orientation")
                else:
                    # Update semantic map with the result
                    store_id = f"store{idx + 1}"
                    rospy.loginfo("[ExploringHandler] Store %d identified: %s", idx + 1, store_type)
                    self.update_store_in_map(store_id, [x, y], store_type)
                    
                    break
            
            # Save semantic map after identifying each store
            self.save_semantic_map()
            
            # Small delay before moving to next location
            rospy.sleep(0.5)
        
        rospy.loginfo("[ExploringHandler] Exploration complete! Visited %d stores", len(self.store_locations))
        rospy.loginfo("[ExploringHandler] Final semantic map saved to %s", self.semantic_map_path)