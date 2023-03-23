#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import ros_numpy

MAX_POINTS = 30000

point_cloud=[[0,0,0,0,0,0,0,0]]
def cloud_callback(msg):
    global point_cloud
    #print msg.point_step
    #print msg.row_step
    #print msg.width
    #print msg.height

    
	# Convert PointCloud2 message to NumPy array
    new_cloud = np.array(list(pc2.read_points(msg)))
    point_cloud=np.concatenate((point_cloud,new_cloud))

    
    #print point_cloud.shape
	
    # Downsample the point cloud using uniform sampling
    num_points = point_cloud.shape[0]
    if num_points > MAX_POINTS:
        step = num_points // MAX_POINTS
        point_cloud = point_cloud[::step, :]

    # Create a new PointCloud2 message with the downsampled points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id
    
    new_msg = PointCloud2()
    new_msg.header=header
    
    new_msg.height = 1
    new_msg.width = point_cloud.shape[0]
    
    new_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
    new_msg.is_bigendian = False
    new_msg.point_step =32
    new_msg.row_step = new_msg.point_step * point_cloud.shape[0]
    new_msg.is_dense = False
    new_msg.data = np.asarray(point_cloud, np.float32).tostring()
    """"""
    #new_msg = msg#pc2.create_cloud(header=header, fields=fields, points=point_cloud)

    # Publish the new point cloud message
    publisher.publish(new_msg)
    
if __name__ == '__main__':
    rospy.init_node('map_generator')

    # Subscribe to the point cloud topic
    subscriber = rospy.Subscriber('/cloud_registered', PointCloud2, cloud_callback)

    # Create a publisher for the downsampled point cloud
    publisher = rospy.Publisher('/map', PointCloud2, queue_size=10)

    rospy.spin()
