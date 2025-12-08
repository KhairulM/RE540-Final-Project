#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os

class StoreMarkerPublisher:
    def __init__(self):
        rospy.init_node('store_marker_publisher', anonymous=False)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.store_coordinates_file = rospy.get_param('~store_coordinates_file', 
            '/home/khairulm/re540/assignment5/assignment5_ws/src/re540_final_map/config/store_coordinates.txt')
        self.marker_scale = rospy.get_param('~marker_scale', 0.3)
        self.marker_height = rospy.get_param('~marker_height', 0.5)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        
        # Publisher
        self.marker_pub = rospy.Publisher('/store_markers', MarkerArray, queue_size=10)
        
        # Load store coordinates
        self.store_coordinates = self._load_coordinates()
        
        rospy.loginfo(f"[StoreMarkerPublisher] Loaded {len(self.store_coordinates)} store locations")
        rospy.loginfo(f"[StoreMarkerPublisher] Publishing to /store_markers at {self.publish_rate} Hz")
        rospy.loginfo(f"[StoreMarkerPublisher] Frame: {self.frame_id}")
        
    def _load_coordinates(self):
        """Load store coordinates from file"""
        coordinates = []
        
        if not os.path.exists(self.store_coordinates_file):
            rospy.logerr(f"[StoreMarkerPublisher] File not found: {self.store_coordinates_file}")
            return coordinates
        
        try:
            with open(self.store_coordinates_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    # Skip comments and empty lines
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse coordinates: (-1.25, -1) or (-1.25,-1)
                    line = line.replace('(', '').replace(')', '').replace(' ', '')
                    parts = line.split(',')
                    
                    if len(parts) == 2:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            coordinates.append((x, y))
                        except ValueError:
                            rospy.logwarn(f"[StoreMarkerPublisher] Invalid coordinate format: {line}")
                            
        except Exception as e:
            rospy.logerr(f"[StoreMarkerPublisher] Error loading coordinates: {e}")
        
        return coordinates
    
    def _create_marker_array(self):
        """Create MarkerArray with all store locations"""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.store_coordinates):
            # Create sphere marker for store location
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "stores"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.marker_height
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            
            # Color (cyan)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(0)  # Persist until deleted
            
            marker_array.markers.append(marker)
            
            # Create text label
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "store_labels"
            text_marker.id = i + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position (slightly above sphere)
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = self.marker_height + self.marker_scale + 0.1
            text_marker.pose.orientation.w = 1.0
            
            # Text
            text_marker.text = f"Store {i+1}\n({x:.2f}, {y:.2f})"
            
            # Scale (text size)
            text_marker.scale.z = 0.15
            
            # Color (white)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.lifetime = rospy.Duration(0)
            
            marker_array.markers.append(text_marker)
        
        return marker_array
    
    def run(self):
        """Main loop to publish markers"""
        rate = rospy.Rate(self.publish_rate)
        
        if not self.store_coordinates:
            rospy.logerr("[StoreMarkerPublisher] No store coordinates to publish!")
            return
        
        marker_array = self._create_marker_array()
        
        rospy.loginfo("[StoreMarkerPublisher] Publishing store markers...")
        
        while not rospy.is_shutdown():
            # Update timestamp
            for marker in marker_array.markers:
                marker.header.stamp = rospy.Time.now()
            
            self.marker_pub.publish(marker_array)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = StoreMarkerPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[StoreMarkerPublisher] Shutting down")
