#!/usr/bin/env python

'''Does not work - Sölvi'''

# Import necessary ROS libraries
import rospy
import tf2_ros
import tf2_py as tf2
import tf2_geometry_msgs
# Import standard ROS message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

# Function to apply transformation to a point
def transform_point(point, transform):
    # Create a PointStamped message from the input point
    point_stamped = PointStamped()
    point_stamped.header = point.header
    point_stamped.point = point.point
    # Apply the transformation
    transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
    # Return only the point from the transformed PointStamped message
    return transformed_point.point

# Callback function for the subscriber
def callback(msg):
    # Apply the transformation to each point in the depth image
    for v in range(msg.height):
        for u in range(msg.width):
            # Get the depth at this pixel
            depth = msg.data[v * msg.width + u]

            # If the depth is not zero (i.e., the point is not at infinity)
            if depth:
                # Create a PointStamped message for this point
                point = PointStamped()
                point.header = msg.header
                point.point.x = u
                point.point.y = v
                point.point.z = depth

                # Transform the point
                transformed_point = transform_point(point, transform)
                # Get the transformed depth
                transformed_depth = transformed_point.z

                # Replace the depth in the original image with the transformed depth
                msg.data[v * msg.width + u] = transformed_depth

    # Publish the transformed image
    pub.publish(msg)

# Initialize the ROS node
rospy.init_node('depth_image_transform')

# Define the publisher to publish transformed depth images
pub = rospy.Publisher('/transformed_aligned_depth_to_color/image_raw', Image, queue_size=1)

# Create a tf2 buffer to store transformations
tf_buffer = tf2_ros.Buffer()

# Create a tf2 listener to listen to transformations
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Create a subscriber to subscribe to depth images
rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callback)

# Continuously lookup for the transformation until the node is manually terminated
while not rospy.is_shutdown():
    try:
        # Lookup for the transformation between the 'map' frame and the 'map_transformed' frame
        transform = tf_buffer.lookup_transform("map", "map_transformed", rospy.Time(0), rospy.Duration(1.0))
    except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
        # Log a warning if the transformation lookup fails
        rospy.logwarn("Failed to find transformation")
    # Sleep for a second before the next lookup
    rospy.sleep(1)

# Keep the node running until it's manually terminated
rospy.spin()
