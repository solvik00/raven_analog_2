#!/usr/bin/env python

# Import necessary ROS and tf2 modules
import rospy
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2

# Define a class for the map cloud transform node
class MapCloudTransform:
    def __init__(self):
        # Initialize a node named 'map_cloud_transform'
        rospy.init_node('map_cloud_transform', anonymous=True)

        # Create a buffer that stores past transforms and a listener that listens to the /tf topic
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the /rtabmap/mapData topic to get the PointCloud2 message
        self.map_data_sub = rospy.Subscriber('/rtabmap/mapData', PointCloud2, self.map_data_callback)
        # Create a publisher that will publish the transformed point cloud on the /shifted_mapData topic
        self.map_data_pub = rospy.Publisher('/shifted_mapData', PointCloud2, queue_size=1)

    # Callback function for the subscriber
    def map_data_callback(self, data):
        try:
            # Lookup for the transform between 'odom' and 'map' frames
            transform = self.tf_buffer.lookup_transform('odom', 'map', rospy.Time())
            # Shift the transformation along the z-axis
            transform.transform.translation.z += 0.37
            # Apply the transformation to the point cloud
            transformed_cloud = do_transform_cloud(data, transform)
            # Publish the transformed point cloud
            self.map_data_pub.publish(transformed_cloud)
        # Catch any exceptions that may be thrown by the lookup_transform function
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as ex:
            rospy.logwarn("Transform error: {}".format(ex))

# The main function
if __name__ == '__main__':
    try:
        # Create an instance of the MapCloudTransform class and spin the node
        mct = MapCloudTransform()
        rospy.spin()
    # Catch the ROSInterruptException exception if the node is shut down
    except rospy.ROSInterruptException:
        pass
