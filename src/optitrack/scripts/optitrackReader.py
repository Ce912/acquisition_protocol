#!/usr/bin/env python
import rospy 
from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import PoseStamped, Transform
from datetime import datetime
import subprocess
import os

#Read messages from optitrack streamer and transform them into robot base frame

#Define the offset between pivot marker and robot'base center
global offset_x, offset_y, offset_z
offset_x = 0.0
offset_y = 0.0
offset_z = 0.0

class OptiTrackSub():
   def __init__(self):
      rospy.init_node('optitrackReader', anonymous=True)  
      self.first_robot_pose = True
      self.PoseStamped_robot = rospy.Subscriber('/optitrack/franka', PoseStamped, self.geometric_transformation, 10)
      self.recording = False

      #Subscribe to raw topics and call a function to convert data in robot base, then publish and save the new topics
      self.pub_glasses = rospy.Publisher('/glasses_new_pose', PoseStamped, queue_size=100)
      self.pub_hand = rospy.Publisher('/hand_new_pose', PoseStamped, queue_size=100)
      self.pub_robot = rospy.Publisher('/robot_base_transform', Transform, queue_size=100)
      self.PoseStamped_glasses = rospy.Subscriber('/optitrack/tobii_glasses',PoseStamped, self.transform_glasses_publisher, 10)
      self.PoseStamped_hand = rospy.Subscriber('/optitrack/hand', PoseStamped,self.transform_hand_publisher, 10)
      self.NewPose_glasses = rospy.Subscriber('/glasses_new_pose', PoseStamped, self.recording_manager, 10)
      self.NewPose_hand = rospy.Subscriber('/hand_new_pose', PoseStamped, self.recording_manager, 10)

   # Transform glasses pose into new frame
   def transform_glasses_publisher(self,data,*args):

      # Get position and orientation as quaternion from the message
      position_g = np.array([[data.pose.position.x],
                            [data.pose.position.y],
                            [data.pose.position.z],
                            [1]], dtype=np.float64)
      rot_g = data.pose.orientation 
      header_glasses = data.header

      # Convert quaternion to rotation matrix
      r_g = R.from_quat([rot_g.x,rot_g.y,rot_g.z,rot_g.w])
      rot_mat_g = r_g.as_matrix()

      # Assembly new trasformation matrix
      glasses_optitrack_frame = np.array([[rot_mat_g[0,0], rot_mat_g[0,1],rot_mat_g[0,2], position_g[0,0]],
                                          [rot_mat_g[1,0], rot_mat_g[1,1],rot_mat_g[1,2], position_g[1,0]],
                                          [rot_mat_g[2,0], rot_mat_g[2,1],rot_mat_g[2,2], position_g[2,0]],
                                          [0,            0,             0,             1]], dtype=np.float64)
   
      try:
         glasses_robot_frame = np.matmul(T_optitrack2robot, glasses_optitrack_frame)
         position_g_ = np.array([[position_g[0,0]],
                              [position_g[1,0]],
                              [position_g[2,0]],
                              [1]], dtype=np.float64)
         # Create new pose messages with the pose in new coordinates and publish them in new  topics. Save those topics in recording manager
         new_pose_glasses = PoseStamped()
         
         # Define the position in the new message considering the offset due to pivot marker position
         new_pose_glasses.pose.position.x = glasses_robot_frame[0,3] + offset_x
         new_pose_glasses.pose.position.y = glasses_robot_frame[1,3] + offset_y
         new_pose_glasses.pose.position.z = glasses_robot_frame[2,3] + offset_z

         rot_robot_frame = np.array([[glasses_robot_frame[0,0], glasses_robot_frame[0,1], glasses_robot_frame[0,2]],
                                    [glasses_robot_frame[1,0], glasses_robot_frame[1,1], glasses_robot_frame[1,2]],
                                    [glasses_robot_frame[2,0], glasses_robot_frame[2,1], glasses_robot_frame[2,2]]], dtype=np.float64)
         
         rot_robot = R.from_matrix(rot_robot_frame)
         rotation_robot_frame = rot_robot.as_quat()

         # Define the orientation in the new message                           
         new_pose_glasses.pose.orientation.x = rotation_robot_frame[0]
         new_pose_glasses.pose.orientation.y = rotation_robot_frame[1]
         new_pose_glasses.pose.orientation.z = rotation_robot_frame[2]
         new_pose_glasses.pose.orientation.w = rotation_robot_frame[3]

         # Define the header in the new message
         new_pose_glasses.header = header_glasses
         
         # Publish new pose on different topic 
         self.pub_glasses.publish(new_pose_glasses)
    
      except Exception as e:
         rospy.logerr("Error creating transformation matrix: %s", e) 
   
   # Transform hand pose into new frame
   def transform_hand_publisher(self,data,*args):
      #Get position and orientation as quaternion from the message
      position_h = np.array([[data.pose.position.x],
                            [data.pose.position.y],
                            [data.pose.position.z]], dtype=np.float64)
      rot_ = data.pose.orientation 
      header_hand = data.header

      #Convert quaternion to rotation matrix
      r_ = R.from_quat([rot_.x, rot_.y, rot_.z, rot_.w])
      rot_mat = r_.as_matrix()

      #Assembly new trasformation matrix
      hand_optitrack_frame = np.array([[rot_mat[0,0], rot_mat[0,1],rot_mat[0,2], position_h[0,0]],
                                        [rot_mat[1,0], rot_mat[1,1],rot_mat[1,2], position_h[1,0]],
                                        [rot_mat[2,0], rot_mat[2,1],rot_mat[2,2], position_h[2,0]],
                                        [0,            0,             0,             1]], dtype=np.float64)
      position_ = np.array([[position_h[0,0]],
                            [position_h[1,0]],
                            [position_h[2,0]],
                            [1]], dtype=np.float64)
      try: 
         hand_robot_frame = np.matmul(T_optitrack2robot, hand_optitrack_frame)
        
         # create new pose messages with the pose in new coordinates 
         new_pose_hand = PoseStamped()
         # Define the position in the new message 
         new_pose_hand.pose.position.x = hand_robot_frame[0,3] + offset_x
         new_pose_hand.pose.position.y = hand_robot_frame[1,3] + offset_y
         new_pose_hand.pose.position.z = hand_robot_frame[2,3] + offset_z

         rot_robot_frame = np.array([[hand_robot_frame[0,0], hand_robot_frame[0,1], hand_robot_frame[0,2]],
                                    [hand_robot_frame[1,0], hand_robot_frame[1,1], hand_robot_frame[1,2]],
                                    [hand_robot_frame[2,0], hand_robot_frame[2,1], hand_robot_frame[2,2]]], dtype=np.float64)
         
         rot_robot = R.from_matrix(rot_robot_frame)
         rotation_robot_frame = rot_robot.as_quat()
                                
         new_pose_hand.pose.orientation.x = rotation_robot_frame[0]
         new_pose_hand.pose.orientation.y = rotation_robot_frame[1]
         new_pose_hand.pose.orientation.z = rotation_robot_frame[2]
         new_pose_hand.pose.orientation.w = rotation_robot_frame[3] 

         #Define the header
         new_pose_hand.header =  header_hand                           
         #Publish new pose message
         self.pub_hand.publish(new_pose_hand)

      except Exception as e:
         rospy.logerr("Error creating transformation matrix: %s", e)

   # Save topics in dedicated bag
   def recording_manager(self,data, *args):
      if not self.recording:
         self.recording = True
         rospy.loginfo("Starting recording...")
         rospy.loginfo("Recording has started")

         # Define parametric name and folder
         date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
         rel_dir = rospy.get_param("~output_dir", ".bags")
         bag_directory = os.path.join(rel_dir, "optitrack")
         bag_name= f"optritrack_bag_{date_time_str}"

         # Define folder for saving raw topics 
         rel_dir = rospy.get_param("~output_dir", ".bags")
         bag_directory2 = os.path.join(rel_dir, "optitrack_raw")
         bag_name2= f"optritrack_raw_bag_{date_time_str}"
         if not os.path.exists(bag_directory):
            os.makedirs(bag_directory)
         if not os.path.exists(bag_directory2):
            os.makedirs(bag_directory2)

         # Change the current working directory
         os.chdir(bag_directory2)
         # Base case: save the raw topics from the optitrack  
         self.bag_process = subprocess.Popen(["rosbag", "record", "-O",bag_name2 , "/optitrack/franka","/optitrack/hand", "/optitrack/tobii_glasses", "/robot_base_transform"])

         # Change the current working directory
         os.chdir(bag_directory)
         # Save the topics with the trasformed poses
         self.bag_process = subprocess.Popen(["rosbag", "record", "-O",bag_name , "/glasses_new_pose","/optitrack/franka", "/hand_new_pose", "glasses_new_pose", "/robot_base_transform"])
         rospy.loginfo(f"Bag started in directory: {bag_directory} with name: {bag_name}.bag")
      
   # Define geometric transformation optitrack2robot_base at first iteration
   def geometric_transformation(self,data,*args ):
      if self.first_robot_pose:
         robot_transform = Transform()

         # Get position
         robot_pose = np.array([[data.pose.position.x],
                              [data.pose.position.y],
                              [data.pose.position.z]],dtype=np.float64)
         # Get orientation
         robot_orient = np.array([[data.pose.orientation.x],
                              [data.pose.orientation.y],
                              [data.pose.orientation.z], 
                              [data.pose.orientation.w]],dtype=np.float64)
         
         #robot_orientation = data.pose.orientation  #4x1
         orientation = R.from_quat(robot_orient)
         orient_ = orientation.as_matrix()
         
            
         #Inverse matrix to get rotation matrix optitrack-to-robot (optitrack orientation in robot base frame)
         rotation_matrix = orient_.T
         tvect = np.matmul(rotation_matrix, robot_pose)
         orient_quat = rotation_matrix.as_quat()
         trasl_vect = -tvect

         robot_transform.translation.x = trasl_vect[0]
         robot_transform.translation.y = trasl_vect[1]
         robot_transform.translation.z = trasl_vect[2]
         robot_transform.rotation.x = orient_quat[0]
         robot_transform.rotation.y = orient_quat[1]
         robot_transform.rotation.z = orient_quat[2]
         robot_transform.rotation.w = orient_quat[3]

         # Publish transformation in a topic
         self.pub_robot.publish(robot_transform)

         global T_optitrack2robot
         T_optitrack2robot = np.array([[rotation_matrix[0,0], rotation_matrix[0,1], rotation_matrix[0,2], -tvect[0,0]],
                                       [rotation_matrix[1,0], rotation_matrix[1,1], rotation_matrix[1,2], -tvect[1,0]],
                                       [rotation_matrix[2,0], rotation_matrix[2,1], rotation_matrix[2,2], -tvect[2,0]],
                                       [0,                     0,                      0,                   1]], dtype=np.float64)
         
         #Compute transform only at first iteration
         self.first_robot_pose = False

      return 


def main():
   node = OptiTrackSub()
   rospy.spin()

if __name__ == '__main__':
   main()