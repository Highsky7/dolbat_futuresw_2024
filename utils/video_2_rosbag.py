#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag
import os

def mp4_to_rosbag(video_path, output_bag_path):
    # Initialize ROS node
    rospy.init_node('video_to_rosbag', anonymous=True)
    
    # Initialize CvBridge
    bridge = CvBridge()
    
    # Open the video file
    cap = cv2.VideoCapture(video_path)
    
    # Check if the video opened successfully
    if not cap.isOpened():
        rospy.logerr(f"Error: Could not open video file {video_path}")
        return

    # Get video properties
    frame_rate = cap.get(cv2.CAP_PROP_FPS)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    rospy.loginfo(f"Video properties - Frame rate: {frame_rate}, Frame size: {frame_width}x{frame_height}, Total frames: {total_frames}")
    
    # Open a ROS bag file for writing
    with rosbag.Bag(output_bag_path, 'w') as bag:
        frame_number = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # Convert the OpenCV image (BGR) to a ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = rospy.Time.from_sec(frame_number / frame_rate)
            ros_image.header.frame_id = "camera"

            # Write the ROS Image message to the bag file
            bag.write('/usb_cam/image_raw', ros_image, ros_image.header.stamp)
            
            rospy.loginfo(f"Writing frame {frame_number}/{total_frames} to bag")
            
            frame_number += 1

    # Release video capture object
    cap.release()
    rospy.loginfo(f"Finished writing to {output_bag_path}")

if __name__ == '__main__':
    try:
        # Set your video path and output bag path here
        current_dir = os.path.dirname(os.path.abspath(__file__))
        video_path = current_dir + '/../resources/track.mp4'
        output_bag_path = current_dir + '/../resources/output.bag'

        if not os.path.exists(video_path):
            rospy.logerr(f"Video file {video_path} does not exist")
        else:
            mp4_to_rosbag(video_path, output_bag_path)
    except rospy.ROSInterruptException:
        pass
