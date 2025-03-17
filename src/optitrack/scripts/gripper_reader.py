#!/usr/bin/env python
import rospy 
import numpy as np
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import Bool
from datetime import datetime
import subprocess
import os

# Simulate the robot gripper (i.e. hand) with the keyboard and save related topics in a bag
class GripperNode():
    def __init__(self):
      rospy.init_node('gripperManager', anonymous=True)  
      
      # Initialize variables
      self.gripper = True
      self.gripper_initial=str(self.gripper)
      self.old_state_gripper = 0
      self.recording = False
      self.listener = Listener(on_press=self.on_press)
      self.listener.start()

      # Set publishing frequency 
      self.gripper_state = rospy.Publisher('gripper_state', Bool, queue_size=10)  #check publishing frequency
      self.gripper_reader = rospy.Subscriber('/gripper_state', Bool, self.recording_manager, 10)
      self.timer = rospy.Timer(rospy.Duration(0.03), self.publish_gripper_state)  #0.1 = 10 Hz
      rospy.loginfo("Current gripper state: %s .Press ESC to change the gripper state", self.gripper_initial)
    
    # Keyboard listener 
    def on_press(self,key):
        if key == Key.esc:
            try:
                # Swith gripper state when pressing the key
                self.old_state_gripper = self.gripper
                self.gripper = not self.gripper
                rospy.loginfo('Esc pressed')
                msg = Bool()
                gripper_state = str(self.gripper)
                self.old_state_gripper = gripper_state
                rospy.loginfo("Current gripper state: %s .Press enter to change the gripper state", gripper_state)
                if self.gripper:  
                    rospy.loginfo('Gripper open')
                else:
                    rospy.loginfo('Gripper closed')
            except AttributeError:
                rospy.logerr('Incorrect input')

    # Publish gripper state
    def publish_gripper_state(self,event):
        msg = Bool()
        msg.data = self.gripper
        self.gripper_state.publish(msg)
        
    # Save topics in a bag 
    def recording_manager(self,data, *args):
        if not self.recording:
            self.recording = True
            rospy.loginfo("Starting recording...")
            rospy.loginfo("Recording has started")

            # Define name and bag saving directory
            date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            rel_dir = rospy.get_param("~output_dir", ".bags")
            bag_directory = os.path.join(rel_dir, "gripper")
            bag_name= f"gripper_bag_{date_time_str}"
            if not os.path.exists(bag_directory):
                os.makedirs(bag_directory)

            #Change the current working directory
            os.chdir(bag_directory)
            
            #Save
            self.bag_process = subprocess.Popen(["rosbag", "record", "-O",bag_name , "/gripper_state"])
            rospy.loginfo(f"Bag started in directory: {bag_directory} with name: {bag_name}.bag")

def main():
   node = GripperNode()
   rospy.spin()


if __name__ == '__main__':
   main()
            
