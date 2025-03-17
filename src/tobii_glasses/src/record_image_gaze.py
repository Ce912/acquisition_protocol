#!/usr/bin/env python
import asyncio
import logging
import os
#import h5py
import pandas as pd
import csv
import numpy as np
from g3pylib import connect_to_glasses 
import dotenv
import rospy
from std_msgs.msg import Bool, String
import cv2
from datetime import datetime
import queue
from threading import Lock, Thread
import signal

#Communicate with tobii glasses and save synchronized frame and gaze.
#Results are stored in a csv with filename and data. Images are saved as .png 

#Initialize Lock
queue_lock = Lock()

#Handles async communication with g3pylib 
class GlassesDataCollector: 
    def __init__(self):
        rospy.init_node("data_collector", anonymous=True)
        rospy.on_shutdown(self.shutdown_handler)

        #Load environment variables
        self.g3_hostname = os.environ["G3_HOSTNAME"] = "tg03b-080203027651.local"
        dotenv.load_dotenv()
        self.data_queue = queue.Queue(maxsize=100000000000)
        self.lock = Lock()

        #Initialize variables
        self.started = False
        self.ended = True
        self.task_queue = queue.Queue(maxsize =1)
        self.is_first = 0
        self.is_last = 0
        self.object_A = ""
        self.object_B = ""
        self.task = False

        # Define subscriber to topics by acquisitionManager node
        rospy.Subscriber("/task", Bool, self.is_first_is_last, 10)
        rospy.Subscriber("/objA", String, self.update_object_A, 10)
        rospy.Subscriber("/objB", String, self.update_object_B, 10)

        #File Setup
        date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_name = f"glasses_images_gaze_{date_time_str}.csv"
        date_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        rel_dir = rospy.get_param("~output_dir", ".bags")
        directory = os.path.join(rel_dir, "gaze_data")
        if not os.path.exists(directory):
            os.makedirs(directory)
        self.file_path = directory + self.file_name

        # CSV header
        self.csv_columns = ['Image name','Gaze_x', 'Gaze_y', ' Frame Timestamp', 'Gaze Timestamp','Initial', 'Final','Object A', 'Object B', 'Height', 'Width', 'Channels']
        self.csv_thread = Thread(target=self.process_queue, daemon = True)
        self.csv_thread.start()

    # Mark initial and final frames for each demo
    def is_first_is_last(self,msg,*args): 
        self.task = msg.data

        if self.task: 
            #Task on
            if self.started == False:   #Start signal not switched yet --> first frame
                self.is_first = 1 
                self.is_last = 0
                self.started = True  #Task started but not ended
                self.ended = False
            else:   #Task on and start signal already switched --> not first frame
                self.is_first = 0
                self.is_last = 0
        
        elif not self.task:  
            #Task off 
            if not self.ended and self.started: #End signal not switched yet
                self.is_last = 1 
                self.ended = True  #Task Ended
                self.started = False #Reset start signal
            else: 
                self.is_last = 0
                self.is_first = 0
        else:
            pass
            
    # Save objects A in csv for completeness        
    def update_object_A(self, msg,*args ):
        self.object_A = msg.data

    # Save objects B in csv for completeness  
    def update_object_B(self, msg, *args):
        self.object_B = msg.data

    #G3pylib async communication
    async def disable_gaze_overlay(self, g3):
        #Disable the gaze overlay on the glasses
        gaze_overlay = await g3.settings.get_gaze_overlay()
        if gaze_overlay:
            success = await g3.settings.set_gaze_overlay(False)
            if success:
                rospy.loginfo("Gaze overlay successfully disabled.")
            else:
                rospy.logwarn("Failed to disable gaze overlay.")
        else:
            rospy.loginfo("Gaze overlay is already disabled.")

    # Record images using Tobii Pro glassespython library
    async def record_frame(self):
        
       #Record frames and gaze coordinates from Tobii glasses  
        async with connect_to_glasses.with_hostname(self.g3_hostname) as g3:
            await self.disable_gaze_overlay(g3)
            async with g3.stream_rtsp(scene_camera=True, gaze=True) as streams:
                async with streams.gaze.decode() as gaze_stream, streams.scene_camera.decode() as scene_stream:
                    
                    # Record the frame
                    while not rospy.is_shutdown():
                        #return  frames
                        logging.info("Recording frame...")
                        frame, frame_timestamp = await scene_stream.get()
                        gaze, gaze_timestamp = await gaze_stream.get()

                        #Align frame and gaze
                        while gaze_timestamp is None or frame_timestamp is None:
                            if frame_timestamp is None:
                                frame, frame_timestamp = await scene_stream.get()
                            if gaze_timestamp is None:
                                gaze, gaze_timestamp = await gaze_stream.get()
                        while gaze_timestamp < frame_timestamp:
                            gaze, gaze_timestamp = await gaze_stream.get()
                            while gaze_timestamp is None:
                                gaze, gaze_timestamp = await gaze_stream.get()

                        # Convert image to array
                        image = frame.to_ndarray(format="bgr24")

                        # Set resolution
                        height, width, channels = [360, 640, 3]
                        resized_image = cv2.resize(image, (width, height)) 
                        
                        # Set output folder for images
                        timestamp = datetime.now().strftime("%d_%H%M%S_%f")[:-3]
                        image_name = f"glasses_frame_" + timestamp + ".jpeg"
                        rel_dir = rospy.get_param("~output_dir", ".bags")
                        image_dir = os.path.join(rel_dir, "gaze_images")
                        if not os.path.exists(image_dir):
                            os.makedirs(image_dir)
                        image_path = os.path.join(image_dir, image_name)

                        # Save resized image
                        cv2.imwrite(image_path, resized_image)

                        # Save gaze coordinates
                        if "gaze2d" in gaze:
                            gaze2d = gaze["gaze2d"]
                            x_2d = int(gaze2d[0] * width)
                            y_2d = int(gaze2d[1] * height)
                        
                        # Build and append new csv row to queue
                        new_data = [image_name, x_2d, y_2d, frame_timestamp, gaze_timestamp, self.is_first, self.is_last, self.object_A, self.object_B, height, width, channels]
                        
                        if not self.data_queue.full():
                            self.data_queue.put(new_data)
                        else:
                            rospy.logwarn("queue full")
        
                        # Frequency of acquisition 10 Hz
                        await asyncio.sleep(0.06)  #
    
    # Processes data from the queue and writes to CSV using pandas 
    def process_queue(self):
            buffer = []
            while not rospy.is_shutdown():
                try:
                    data = self.data_queue.get(timeout=1)  # Wait for data
                    buffer.append(data)

                    # Write in batches of 10000 for efficiency
                    if len(buffer) >= 10000:
                        self.flush_buffer(buffer)
                        rospy.loginfo("10000 rows")
                    self.data_queue.task_done()
            
                except queue.Empty:
                    rospy.loginfo("Queue empty")
                    pass  # No new data, continue checking

            # Flush remaining data when shutting down
            if buffer:
                self.flush_buffer(buffer)
                rospy.loginfo("Queue still present")
            #self.data_queue.task_done()
            rospy.loginfo("Queue saved.end")
        
    # Ensure remaining buffer is written before shutdown        
    def shutdown_handler(self):
        rospy.loginfo("Shutting down, flushing remaining data...")
        if not self.data_queue.empty():
            self.flush_buffer(list(self.data_queue.queue))  # Flush all remaining data
        self.csv_thread.join()  # Ensure CSV thread finishes before exiting
        rospy.loginfo("All data saved successfully.")

    # Writes buffered data to CSV using pandas
    def flush_buffer(self, buffer):
        with self.lock:
            df = pd.DataFrame(buffer, columns=self.csv_columns)
            df.to_csv(self.file_path, mode='a', header=not os.path.exists(self.file_path), index=False)
            buffer.clear()  # Empty buffer after writing

    # Main data collection function
    async def collect_data(self):
        #os.environ["G3_HOSTNAME"] = "tg03b-080203027651.local"
        try: 
            rospy.loginfo("Starting glasses data collection...")
            await self.record_frame()
            
        except Exception as e:
            rospy.logerr(f"Error during data collection")
            rospy.loginfo(e)
        finally:
            rospy.loginfo(f"Demonstrations collected. Dataset saved")

# Main entry point
if __name__ == "__main__":
    try:
        collector = GlassesDataCollector()
        asyncio.run(collector.collect_data())
        def signal_handler(sig, frame):
            rospy.loginfo("Received shutdown signal, flushing data...")
            collector.shutdown_handler()
            rospy.signal_shutdown("Graceful exit")

        signal.signal(signal.SIGINT, signal_handler)    
        signal.signal(signal.SIGTERM, signal_handler)
    except rospy.ROSInterruptException:
        rospy.loginfo("Data collection interrupted.")
   