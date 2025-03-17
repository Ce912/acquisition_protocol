#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from datetime import datetime
import subprocess
import os

#Read topic from a realsense and save them in a dedicated topic
class RealSenseSub():
   def __init__(self):
      
      rospy.init_node('RS_externalReader', anonymous=True)
      #Initialize recording and subscriber
      self.recording = False

      self.ImageCol = rospy.Subscriber('ImageCol_Ext', Image, self.recording_manager, 10)
      # RGBD topics publish rgb and depth in the same message. It works only for depth aligned to color images (synchronized by frame time tag).
      self.RGBDCameraInfo= rospy.Subscriber('RGBDCameraInfo_Ext', CameraInfo, self.recording_manager, 10)
      self.RGBDImage= rospy.Subscriber('RGBDImage_Ext', Image, self.recording_manager, 10)

   # Start the recording 
   def recording_manager(self,data, *args):
      if not self.recording:
         self.recording = True
         rospy.loginfo("Starting recording for exteranl camera...")
         rospy.loginfo("External camera recording has started")

         #Set name for bag and folder
         date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
         rel_dir = rospy.get_param("~output_dir", ".bags")
         bag_directory = os.path.join(rel_dir, "RS")
         bag_name= f"RSexternal_bag_{date_time_str}"
         if not os.path.exists(bag_directory):
            os.makedirs(bag_directory)

         # Change the current working directory
         os.chdir(bag_directory)
         buffer_size = "2048"
         self.bag_process = subprocess.Popen(["rosbag", "record", "-b", buffer_size, "-O",bag_name, "/RGBDCameraInfo_Ext", "/RGBDImage_Ext", "/ImageCol_Ext"])
         rospy.loginfo(f"Bag started in directory: {bag_directory} with name: {bag_name}.bag")

def main():
   node= RealSenseSub()
   rospy.spin()

if __name__ == '__main__':
   main()
    
    
    


    