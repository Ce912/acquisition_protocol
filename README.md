# acquisition_protocol
This repo contains an efficient acquisition protocol for acquiring robotic dataset handling multiple sensors (Realsense and Femto camera, Tobii Pro Glasses, OptiTrack). 
Each sensor is managed by a dedicated package (in /src) and its topics are saved in a separated bag. 
The acquisition is handled by the AcquisitionManager node in the /acquisition package, which provides a vocal assistant for simple temporized acquisition. 
Refer to the relative config file for setting the parameters (/acquisition/config/). 
Prior acquisition, cameras must be calibrated and tobii glasses properly connected and calibrated. 
Topics stream from optitrack must be available before starting the pipeline.

NOTE: A set of 50 demo produces 50 GB of data. Make sure you're storing file in an appropriate location (i.e. external disk). Set the path in acquisition/config/acquisition_config.yaml  (i.e. "output_dir")

# Start the pipeline
To launch the system, three terminals are needed. 
It's recommended to launch cameras first due to high latency. Wait for the RealSense node activation before launching other cameras.
Follow this order for avoiding any buffer issue. 

Terminal 1: 
> roslaunch realsense test_launch.launch 
This file will launch RS camera, the gripper simulator, the glasses data collector and the optitrack reader.
For handling the gripper, (i.e. switch True/False: True = gripper open, False = gripper closed ), press the relative key ("Esc", by default).
Add a terminal with rostopic echo /gripper_state to check the gripper status in realtime

Terminal 2: 
> roslaunch realsense test_launch_femto.launch 
Wait for Femto Node activation before launching other cameras

Terminal 3: 
> roslaunch realsense test_launch2.launch 
This launch file will start the acquisition manager node. Be ready for the acquisition!

REMINDER: If you want to collect more than a set, change the output folder name ("bags" by default) after each round to avoid overwriting. 

# Data processing
A ready-to-use processing matlab script is available in the /data_processing folder. For seamless usage, keep the original names of the subfolders (in /bags). The script reads a sigle "bags" folder, extract the ros messages and synchronize the data according to the camera streams [/synchronizeData function].
The script outputs two .csv files, one with the information from the glasses, the other one with the remaining data. 
Check the "createGlassesDataset.m" and "createDataset.m" functions for additional details. 






