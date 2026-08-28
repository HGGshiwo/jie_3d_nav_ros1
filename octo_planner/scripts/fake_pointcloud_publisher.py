#!/usr/bin/env python3
import rospy
import random
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

def publish_fake_pc():
    rospy.init_node('fake_pointcloud_publisher', anonymous=True)
    
    topic = rospy.get_param('~topic', '/lidar_points')
    frame_id = rospy.get_param('~frame_id', 'map')  # Using 'map' avoids needing active TF transforms
    rate_hz = rospy.get_param('~rate', 10.0)
    
    pub = rospy.Publisher(topic, PointCloud2, queue_size=10)
    rate = rospy.Rate(rate_hz)
    
    rospy.loginfo(f"Starting fake point cloud publisher on topic: {topic} (frame: {frame_id}) at {rate_hz} Hz")
    
    # Pre-generate some obstacle columns and ground points with high density (~30,000 points)
    base_points = []
    
    # 1. High density Ground plane (X: -6.0 to 6.0, Y: -6.0 to 6.0, step 0.08m) -> ~22,800 points
    for ix in range(-75, 76):
        x = ix * 0.08
        for iy in range(-75, 76):
            y = iy * 0.08
            base_points.append((x, y, 0.0))
            
    # 2. 4 High density columns -> ~6,500 points
    import math
    centers = [(2.0, 2.0), (-2.0, 2.0), (2.0, -2.0), (-2.0, -2.0)]
    for cx, cy in centers:
        # Height from 0.0 to 2.5m, step 0.05m (51 layers)
        for iz in range(0, 51):
            z = iz * 0.05
            # Circular layer with 32 points
            for iang in range(32):
                angle = iang * (2.0 * math.pi / 32.0)
                px = cx + 0.3 * math.cos(angle)
                py = cy + 0.3 * math.sin(angle)
                base_points.append((px, py, z))
        
    while not rospy.is_shutdown():
        # Add slight random noise to simulate real sensor variation
        points = []
        for x, y, z in base_points:
            nx = x + random.uniform(-0.02, 0.02)
            ny = y + random.uniform(-0.02, 0.02)
            nz = z + random.uniform(-0.01, 0.01)
            points.append((nx, ny, nz))
            
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        pc_msg = pc2.create_cloud_xyz32(header, points)
        pub.publish(pc_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_pc()
    except rospy.ROSInterruptException:
        pass
