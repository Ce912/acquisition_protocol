#!/usr/bin/env python
import rospy 


from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D
from std_msgs.msg import Float32

from datetime import datetime

import subprocess
import os

class OptiTrackSub():
   def __init__(self):
        rospy.init_node('optitrackReader', anonymous=True)
        self.Odometry = rospy.Subscriber('odom', Odometry,self.recording_manager, 10)
        self.PoseStamped = rospy.Subscriber('pose', PoseStamped,self.recording_manager, 10)
        self.Pose2D = rospy.Subscriber('pose2D',Pose2D,self.recording_manager, 10)

        self.prova= rospy.Subscriber('rndm_number', Float32 ,self.recording_manager,10 )
        self.recording = False

        


   def recording_manager(self,data, *args):
      if not self.recording:
         self.recording = True
         rospy.loginfo("Starting recording...")
         print("Recording has started")

         date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
         bag_directory = "./bags"
         bag_name= f"optritrack_bag_{date_time_str}"

         if not os.path.exists(bag_directory):
            os.makedirs(bag_directory)

        # Change the current working directory
         os.chdir(bag_directory)

         self.bag_process = subprocess.Popen(["rosbag", "record", "-O",bag_name , "/odom /pose /pose2D"])
         print(f"Bag started in directory: {bag_directory} with name: {bag_name}.bag")

def main():
   node= OptiTrackSub()
   rospy.spin()
   #node.destroy_node()
   #rospy.is_shutdown()

if __name__ == '__main__':
   main()