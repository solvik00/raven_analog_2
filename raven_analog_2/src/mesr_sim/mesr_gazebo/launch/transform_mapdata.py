# Import necessary ROS libraries
import rospy
import tf2_ros
import tf2_py as tf2
import tf2_sensor_msgs
# Import custom message type from rtabmap_ros package
from rtabmap_ros.msg import MapData
# Import standard ROS message types
from sensor_msgs.msg import PointCloud2

# Define callback function for subscriber
def callback(msg):
    # Check if cloud_map is present in the message
    if msg.cloud_map:
        try:
            # Apply the static transformation to the point cloud
            transformed_pointcloud = tf_buffer.transform(msg.cloud_map, "map_transformed", rospy.Duration(1.0))
            # Publish the transformed point cloud
            pub.publish(transformed_pointcloud)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            # Log a warning if the transformation fails
            rospy.logwarn("Failed to transform point cloud")

# Initialize the ROS node
rospy.init_node('mapdata_transform')

# Define the publisher to publish transformed map data
pub = rospy.Publisher('/transformed_map_data', PointCloud2, queue_size=1)

# Create a tf2 buffer to store transformations
tf_buffer = tf2_ros.Buffer()

# Create a tf2 listener to listen to transformations
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Create a subscriber to subscribe to MapData messages
rospy.Subscriber('/rtabmap/mapData', MapData, callback)

# Keep the node running until it's manually terminated
rospy.spin()
