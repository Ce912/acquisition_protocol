#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
from pynput.keyboard import Key, Listener, KeyCode
import subprocess
import os
import random
import time
import yaml
from datetime import datetime
import pyttsx3

#The AcquisitionManager handles the temporized acquisition of a set of ndemo demonstrations, randomly generated from the objects list. 
#The vocal assistant reads the name of the task and start/signals. 
#Topics are saved in a bag (./bags/manager), check the config/acquisition_config.yaml for changing directory and parameters

class AcquisitionManager():
    def __init__(self):
        rospy.init_node('acquisition_manager', anonymous=True) 
        
        #Get parameters 
        self.Ndemo = rospy.get_param("~Ndemo")
        self.active_time = rospy.get_param("~active_time")
        self.idle_time = rospy.get_param("~idle_time")

        #Initialize empty variables
        self.obj_A = ""
        self.obj_B = ""
        self.recording = False
        self.task = False

        # Publishers and timers for publishing frequency
        self.task_state = rospy.Publisher('/task', Bool, queue_size=10 )
        self.object_A = rospy.Publisher('/objA', String, queue_size=10 )
        self.object_B = rospy.Publisher('/objB', String, queue_size=10 )
        self.timer1 = rospy.Timer(rospy.Duration(0.06), self.publish_task)
        self.timer2 = rospy.Timer(rospy.Duration(0.06), self.publish_objects)
        self.timer_saving = rospy.Timer(rospy.Duration(0.1), self.recording_manager)
  
        # Get static and dynamic objects names
        static_obj = rospy.get_param("~static_obj")
        dynamic_obj = rospy.get_param("~dynamic_obj")

        # Define N random tasks
        self.objs_B = random.choices(static_obj, k = self.Ndemo)
        self.objs_A = random.choices(dynamic_obj, k = self.Ndemo)

        # Save Ndemo, objA, objB sequence in a yaml file for backup
        self.data_saving()
        self.engine = pyttsx3.init()

        # Set up the vocal assistant
        self.engine.setProperty('rate', 120)
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[20].id)

        # Start acquisition 
        self.acquisition_timer()
        rospy.on_shutdown(self.shutdown)

    # Saving Ndemo and objects A , objects B list in a yaml file for backup
    def data_saving(self):
        date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        directory = "./src/acquisition/task_info"
        if not os.path.exists(directory):
            os.makedirs(directory)
        file_name = f"task_info_{date_time_str}.yaml"
        output_file = os.path.join(directory, file_name)

        info = {"Number of demonstration " : self.Ndemo, 
                    "objects A list ": self.objs_A, 
                    "objects B list ": self.objs_B,
                }
        with open(output_file, "w") as f:
            yaml.dump(info, f, default_flow_style=False, sort_keys=False)

        rospy.loginfo(f"Tasks infos saved in {output_file}")
        
    # Handle task variable (on/off) according to temporized acquisition
    def acquisition_timer(self):
        for idx in range(self.Ndemo):
            rate = rospy.Rate(1)
            self.obj_A = self.objs_A[idx]
            self.obj_B = self.objs_B[idx]
            rospy.loginfo(f"Perform task: {self.obj_A} into {self.obj_B}")

            for sec in range(self.idle_time,0,-1):
                rospy.loginfo(f"Acquisition starts in {sec} seconds")
                rate.sleep()

            text_to_speak = f"Task: {self.obj_A} into {self.obj_B}!"
            self.engine.say(text_to_speak)
            self.engine.runAndWait()
            rate.sleep()
            time.sleep(3)
            text_to_speak = f"Start!"
            self.engine.say(text_to_speak)
            self.engine.runAndWait()
            rospy.loginfo(f"Acquisition starts now!")
            self.task = True

            for sec in range(self.active_time,0,-1):
                rospy.loginfo(f"Acquisition started, remaining {sec} seconds")
                rate.sleep()
                
            counter = idx+1
            self.task = False
            text_to_speak = f"Stop!"
            self.engine.say(text_to_speak)
            self.engine.runAndWait()
            rate.sleep()
            rospy.loginfo(f"Acquisition {counter} completed!")
        
        rospy.loginfo(f"Data collection procedure completed")

    # Publish task topic 
    def publish_task(self,event):
        msg = Bool()
        msg.data = self.task
        self.task_state.publish(msg)

    # Publish objects names into respective topics
    def publish_objects(self,event):
        msgA = String()
        msgA.data = self.obj_A
        self.object_A.publish(msgA)
        msgB = String()
        msgB.data = self.obj_B
        self.object_B.publish(msgB)

    # Check if process ir running before shutting down
    def shutdown(self):
        rospy.loginfo("Shutting down...")
        if self.bag_process.poll() is None:
            self.bag_process.terminate()
            self.bag_process.wait()
        rospy.loginfo("ROS bag recording stopped.")

    # Save a bag with parametric name in the "bags/manager" folder
    def recording_manager(self,data, *args):
        if not self.recording:
            self.recording = True
            rospy.loginfo("Starting recording...")
            rospy.loginfo("Recording has started")

            # Define name and bag saving directory
            date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            rel_dir = rospy.get_param("~output_dir", ".bags")
            bag_directory = os.path.join(rel_dir, "manager")
            bag_name= f"manager_bag_{date_time_str}"
            if not os.path.exists(bag_directory):
                os.makedirs(bag_directory)

            # Change the current working directory
            os.chdir(bag_directory)

            # Save a bag with relevant topics
            self.bag_process = subprocess.Popen(["rosbag", "record", "-O",bag_name , "/task", "/objA", "/objB"])
            rospy.loginfo(f"Bag started in directory: {bag_directory} with name: {bag_name}.bag")


def main():
   node = AcquisitionManager()
   rospy.spin()


if __name__ == '__main__':
   main()
