#!/usr/bin/env python3
import rospy
import yaml
import os
import cv2
import tempfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pilot.srv import ChatCompletion, IdentifyStore, IdentifyStoreResponse
from geometry_msgs.msg import PoseStamped
import tf


class SemanticMapBuilderNode:
    def __init__(self):
        rospy.init_node('semantic_map_builder')
        
        # Parameters
        self.map_file = rospy.get_param("~semantic_map_file", 
                                       os.path.join(os.path.dirname(__file__), 
                                                   "../config/semantic_map.yaml"))
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.identification_prompt = rospy.get_param("~identification_prompt",
                                                    "You are a store identification system for a shopping robot. "
                                                    "Look at this store front image and identify what type of store it is. "
                                                    "Respond with ONLY the store type in 1-3 words (e.g., 'grocery store', 'electronics', 'clothing', 'pharmacy', etc.). "
                                                    "Be specific but concise.")
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        
        # Store latest camera image
        self.latest_image = None
        self.latest_image_time = None
        
        # Load semantic map
        self.semantic_map = self._load_semantic_map()
        
        # Wait for LLM chat service
        rospy.loginfo("[SemanticMap] Waiting for /llm_chat service...")
        rospy.wait_for_service('/llm_chat', timeout=30)
        self.llm_chat = rospy.ServiceProxy('/llm_chat', ChatCompletion)
        rospy.loginfo("[SemanticMap] Connected to LLM service")
        
        # Subscribe to camera
        self.image_sub = rospy.Subscriber(
            self.camera_topic, 
            Image, 
            self.image_callback,
            queue_size=1
        )
        
        # Service for store identification
        self.identify_service = rospy.Service(
            'identify_store',
            IdentifyStore,
            self.handle_identify_store
        )
        
        rospy.loginfo("[SemanticMap] Semantic Map Builder initialized")
        rospy.loginfo("[SemanticMap] Subscribing to: %s", self.camera_topic)
        rospy.loginfo("[SemanticMap] Map file: %s", self.map_file)
        rospy.loginfo("[SemanticMap] Service ready: /identify_store")
    
    def _load_semantic_map(self):
        """Load semantic map from YAML file."""
        if os.path.exists(self.map_file):
            try:
                with open(self.map_file, 'r') as f:
                    map_data = yaml.safe_load(f)
                    if map_data is None:
                        map_data = {}
                    rospy.loginfo("[SemanticMap] Loaded map with %d stores", len(map_data))
                    return map_data
            except Exception as e:
                rospy.logerr("[SemanticMap] Failed to load map: %s", str(e))
                return {}
        else:
            rospy.logwarn("[SemanticMap] Map file not found, starting with empty map")
            return {}
    
    def _save_semantic_map(self):
        """Save semantic map to YAML file."""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.map_file), exist_ok=True)
            
            with open(self.map_file, 'w') as f:
                yaml.dump(self.semantic_map, f, default_flow_style=False)
            rospy.loginfo("[SemanticMap] Map saved successfully")
            return True
        except Exception as e:
            rospy.logerr("[SemanticMap] Failed to save map: %s", str(e))
            return False
    
    def image_callback(self, msg):
        """Store latest camera image."""
        self.latest_image = msg
        self.latest_image_time = rospy.Time.now()
    
    def handle_identify_store(self, req):
        """Handle store identification service request."""
        response = IdentifyStoreResponse()
        
        try:
            # Validate store name
            if not req.store_name or req.store_name.strip() == "":
                response.success = False
                response.message = "Store name cannot be empty"
                rospy.logwarn("[SemanticMap] %s", response.message)
                return response
            
            store_name = req.store_name.strip()
            
            # Check if we have a recent image
            if self.latest_image is None:
                response.success = False
                response.message = "No camera image available"
                rospy.logwarn("[SemanticMap] %s", response.message)
                return response
            
            # Check image age
            image_age = (rospy.Time.now() - self.latest_image_time).to_sec()
            if image_age > 5.0:
                rospy.logwarn("[SemanticMap] Using old image (%.1f seconds old)", image_age)
            
            rospy.loginfo("[SemanticMap] Identifying store: %s", store_name)
            
            # Convert ROS image to OpenCV format
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            except Exception as e:
                response.success = False
                response.message = f"Image conversion failed: {str(e)}"
                rospy.logerr("[SemanticMap] %s", response.message)
                return response
            
            # Save image to temporary file
            temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
            temp_path = temp_file.name
            temp_file.close()
            
            try:
                cv2.imwrite(temp_path, cv_image)
                rospy.logdebug("[SemanticMap] Image saved to: %s", temp_path)
                
                # Call LLM service for identification
                llm_response = self.llm_chat(
                    prompt=self.identification_prompt,
                    system_message="",
                    temperature=0.3,
                    max_tokens=50,
                    image_path=temp_path,
                    text_file_path="",
                    image_url="",
                    use_vision_model=True
                )
                
                if not llm_response.success:
                    response.success = False
                    response.message = f"LLM identification failed: {llm_response.error_message}"
                    rospy.logerr("[SemanticMap] %s", response.message)
                    return response
                
                # Extract store type from LLM response
                store_type = llm_response.response.strip()
                
                # Update semantic map
                if store_name not in self.semantic_map:
                    self.semantic_map[store_name] = {
                        'location': [0.0, 0.0],  # Default location
                        'type': ''
                    }
                
                self.semantic_map[store_name]['type'] = store_type
                
                # Save map
                if self._save_semantic_map():
                    response.success = True
                    response.store_type = store_type
                    response.message = f"Store '{store_name}' identified as '{store_type}'"
                    rospy.loginfo("[SemanticMap] %s", response.message)
                else:
                    response.success = False
                    response.message = "Failed to save map"
                    rospy.logerr("[SemanticMap] %s", response.message)
            
            finally:
                # Clean up temporary file
                try:
                    os.unlink(temp_path)
                except:
                    pass
        
        except Exception as e:
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            rospy.logerr("[SemanticMap] %s", response.message)
        
        return response
    
    def get_robot_pose(self):
        """Get current robot pose from TF."""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                '/map', '/base_link', rospy.Time(0)
            )
            return trans[0], trans[1]  # x, y
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[SemanticMap] Could not get robot pose from TF")
            return None, None
    
    def update_store_location(self, store_name):
        """Update store location based on current robot pose."""
        x, y = self.get_robot_pose()
        if x is not None and y is not None:
            if store_name in self.semantic_map:
                self.semantic_map[store_name]['location'] = [float(x), float(y)]
                self._save_semantic_map()
                rospy.loginfo("[SemanticMap] Updated location for %s: [%.2f, %.2f]", 
                             store_name, x, y)
    
    def spin(self):
        """Keep node running."""
        rospy.spin()


def main():
    try:
        node = SemanticMapBuilderNode()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SemanticMap] Node shutting down")
    except Exception as e:
        rospy.logerr("[SemanticMap] Fatal error: %s", str(e))


if __name__ == "__main__":
    main()
