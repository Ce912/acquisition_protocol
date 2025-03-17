#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage, Image
import message_filters
from datetime import datetime

import subprocess
import os

#FemtoReader node: reads topics of interest and save them in a dedicated bag
class FemtoSub():
   def __init__(self): 
      rospy.init_node('Femto_Reader', anonymous=True)

      #Initialization
      self.recording = False

      # RGBD topics publish rgb and depth in the same message. It works only for depth aligned to color images (synchronized by frame time tag).
      # If it works, no need of acquiring the topics listed below
      #self.RGBDCameraInfo= rospy.Subscriber('RGBDCameraInfo_Ext', CameraInfo, self.recording_manager, 10)
      #self.RGBDImage= rospy.Subscriber('RGBDImage_Ext', Image, self.recording_manager, 10)
      
      #Use a filter to synchronize color and depth frames if no aligned topics are available for the device
      self.CameraInfoColor = message_filters.Subscriber('/CameraInfo',CameraInfo)
      self.ImageColor = message_filters.Subscriber('/ImageColor',CompressedImage)
      self.ImageDepth = message_filters.Subscriber('/ImageDepth',Image)
      sync = message_filters.ApproximateTimeSynchronizer([self.ImageDepth, self.CameraInfoColor, self.ImageColor], 
         queue_size=10, 
         slop=0.05)  # Allow up to 0.05 seconds of timestamp difference
      sync.registerCallback(self.recording_manager)

   #Save the topics in a bag
   def recording_manager(self,data, *args):
      if not self.recording:
         self.recording = True
         rospy.loginfo("Starting recording for femto camera...")

         # Define name and bag saving directory
         date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
         rel_dir = rospy.get_param("~output_dir", ".bags")
         bag_directory = os.path.join(rel_dir, "femto")
         bag_name= f"femto_bag_{date_time_str}"
         if not os.path.exists(bag_directory):
            os.makedirs(bag_directory)

         # Change the current working directory
         os.chdir(bag_directory)
         buffer_size = "2048"

         self.bag_process = subprocess.Popen(["rosbag", "record","-b", buffer_size, "-O",bag_name, "/CameraInfo", "/ImageColor", "/ImageDepth"])
         rospy.loginfo(f"Bag started in directory: {bag_directory} with name: {bag_name}.bag")

def main():
   node = FemtoSub()
   rospy.spin()
if __name__ == '__main__':
   main()
    
    
    


    