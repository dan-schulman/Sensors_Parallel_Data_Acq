# -*- coding: utf-8 -*-
"""
ME499 Lab 4: Sensors
University of Michigan
Winter 2021
Code by: Daniel Sousa Schulman - dschul@umich.edu
Professor: Shorya Awtar

The following code collects serial angle data from an Arduino (Encoder and IMU) and video data from a USB camera.
The code uses AruCo markers to estimate angular displacement of the marker.
It plots angular displacement of AruCo markers along with serial incoming Arduino Encoder angle data and IMU angle data.  
"""

# calling libraries and renaming them so its easy to refer to them in the rest of the code
#%matplotlib qt5
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import csv
from scipy.spatial.transform import Rotation as R
import time
from threading import Thread 
import queue
import serial
import matplotlib.pyplot as plt
from scipy import signal


class SerialObj: #class used to handle the arduino serial commm. using parallel computation
   def __init__(self, COM,filename_Ard, baud=115200):
       self.ser = serial.Serial(COM,baud) #start the serial port
       time.sleep(0.25)
       self.stopped = False      
       self.ser.flush() #flush the buffer
       self.ser.reset_input_buffer()
       self.ser.reset_output_buffer()
       time.sleep(0.25)
       self.Q = queue.Queue(maxsize=50) #max size for the data queue
       prev1 = open(filename_Ard + '.csv', 'a', newline='') #create a file to save the raw data
       #this will erase if a file already exists with this name
       prev1.seek(0)
       prev1.truncate()
       prev1.close()
       time.sleep(2)
       self.ser.write(b's') #arduino waits for 's' character to start acquisition
       self.sync_time = time.time_ns() #this will be used to sync the time arcoss devices in the future
    
   def start(self):
   		# start a thread to read frames from the file video stream
   		t = Thread(target=self.update, args=()) #starting a parallel thread. this makes the program run faster. Relevant in Part 2/3
   		t.daemon = True
   		t.start()
   		return self
    
   def update(self):
        #this function keeps updating the dataqueue by reading serially indiefinetely
   		self.counter = 0
        #t1 = time.time_ns()
   		while True:
   			self.counter += 1
               # if the thread indicator variable is set, stop the thread
   			if self.stopped:
   				return
   			# otherwise, ensure the queue has room in it
   			if not self.Q.full():
   				line = self.ser.readline().decode('utf-8') #decode serial message
   				
   				line = line.rstrip().lstrip() #get rid of \r and \n characters
   				ard_data = line.split(",") #creates an array from string separated by commas
   				with open(filename_Ard + '.csv', 'a', newline='') as file: #this writes the raw arduino data to the csv file
   				   writer = csv.writer(file)
   				   writer.writerow(ard_data)
   				self.Q.put(ard_data) #put the newly acquired data in the queue
        
   def read(self):
   		# return next data in the queue
   		return self.Q.get()
        
   def more(self):  		
       # return True if there are still data in the queue       
   		return self.Q.qsize() > 0
           
   def stop(self):
   		# indicate that the thread should be stopped
   		self.ser.close() 
   		self.stopped = True

class FileVideoStream: #class used to handle camera video capture through parallel computation
   def __init__(self, path, queueSize=12):
  		# initialize the file video stream along with the boolean
  		# used to indicate if the thread should be stopped or not
        self.stream = cv.VideoCapture(0,cv.CAP_DSHOW)
        
        # configure FPS sampling and exposure
        self.stream.set(cv.CAP_PROP_FPS, 20) 
        self.stream.set(cv.CAP_PROP_AUTO_EXPOSURE, -6)
        self.stopped = False
        # initialize the queue used to store frames read from the video file
        self.Q = queue.Queue(maxsize=queueSize)
        
   def start(self):
  		# start a thread to read frames from the file video stream
  		t = Thread(target=self.update, args=())
  		t.daemon = True
  		t.start()
  		return self
    
   def update(self):
  		# keep looping infinitely
  		while True:
  			# if the thread indicator variable is set, stop the thread
  			if self.stopped:
  				return
  			# otherwise, ensure the queue has room in it
  			if not self.Q.full():
  				# read the next frame from the file
  				(grabbed, frame) = self.stream.read()
  				# if the `grabbed` boolean is `False`, then we have
  				# reached the end of the video file
  				if not grabbed:
  					self.stop()
  					return
  				# add the frame to the queue
  				self.Q.put(frame)
    
   def read(self):
  		# return next frame in the queue
  		return self.Q.get()
    
   def more(self):
  		# return True if there are still frames in the queue
  		return self.Q.qsize() > 0

   def stop(self):
  		# indicate that the thread should be stopped 		
          self.stopped = True

class LowPass:
    def __init__(self, fc, fs):
        w0 = 2*np.pi*fc
        self.tau = 1/w0
        num = w0
        den = [1,w0] #TF = wc/(s+wc)
        lowPass = signal.TransferFunction(num,den) #continuous TF
        dt = 1/fs
        discreteLowPass = lowPass.to_discrete(dt,method='gbt',alpha=0.5) #tustin method - bilinear approx 
        self.b = discreteLowPass.num; #b[0]*z+b[1]
        self.a = -discreteLowPass.den; #a[0]*z+a[1]
        self.a = [float(k) for k in self.a]
        self.b = [float(k) for k in self.b]
        self.xfilt = [0.0]
        self.x = [0.0]
    
    def update(self,xnew):
        self.x.append(xnew)
        nf = len(self.xfilt)
        n = len(self.x)
        #Update law for LPF
        self.xfilt.append(self.a[1]*self.xfilt[nf-1] + self.b[0]*self.x[n-1] + self.b[1]*self.x[n-2])
        return self.xfilt[nf]
        
class Integrator:
    def __init__(self, fs):
        self.dt = 1/fs
        self.xInt = [0.0]
        self.x = [0.0]
        
    def update(self,xnew):
        self.x.append(xnew)
        nf = len(self.xInt)
        n = len(self.x) 
        #trapezoidal rule
        self.xInt.append(self.xInt[nf-1] + (self.dt/2)*self.x[n-1] + (self.dt/2)*self.x[n-2])
        return self.xInt[nf]

class HighPass:
    def __init__(self, fc, fs):
        w0 = 2*np.pi*fc
        self.tau = 1/w0
        num = [1,0]
        den = [1,w0] #TF = 1/(s+wc)
        highPass = signal.TransferFunction(num,den) #cont TF
        dt = 1/fs
        discreteHighPass = highPass.to_discrete(dt,method='gbt',alpha=0.5) #tustin method - bilinear approx
        self.b = discreteHighPass.num;
        self.a = -discreteHighPass.den;
        self.a = [float(k) for k in self.a]
        self.b = [float(k) for k in self.b]
        self.xfilt = [0.0]
        self.x = [0.0]
    
    def update(self,xnew):
        self.x.append(xnew)
        nf = len(self.xfilt)
        n = len(self.x) 
        #update law for HPF
        self.xfilt.append(self.a[1]*self.xfilt[nf-1] + self.b[0]*self.x[n-1] + self.b[1]*self.x[n-2])
        return self.xfilt[nf]

       

filename = 'test_DS_cam1' #this is the camera file name
filename_Ard = 'test_DS_Ard1' #this is the raw arduino data file name
filename_Proc_Ard = 'test_DS_Ard_Proc1' #this is the processed arduino data file name

prev = open(filename + '.csv', 'a', newline='') #this will erase if a file already exists with this name
prev.seek(0)
prev.truncate()
prev.close()


prev1 = open(filename_Proc_Ard + '.csv', 'a', newline='') #this will erase if a file already exists with this name
prev1.seek(0)
prev1.truncate()
prev1.close()



LPF = LowPass(3,40) #Crete the LPF. fc in Hz, fs in Hz. Change fc and fs accordingly
HPF = HighPass(3,40)
Int = Integrator(40) #Create digital integrator with fs

fvs = FileVideoStream(1) #the number is the camera path. for my laptop 0 = internal, 1 = external

# Set calibration file source, change name based on camera
cv_file = cv.FileStorage("calibration_Camera_1.yaml", cv.FILE_STORAGE_READ)

# Retrieve calibration values
cmatrix = cv_file.getNode("camera_matrix").mat() # intrinsic camera factor
dmatrix = cv_file.getNode("dist_coeff").mat() # distortion matrix (intrinsic camera factor)
arucoID = 0 #select the aruco id


# Print calibration values
print("cmatrix :", cmatrix.tolist())
print("dmatrix :", dmatrix.tolist())

# Relinquish calibration file access
cv_file.release()

#create serial object
ser = SerialObj('COM14',filename_Ard,115200)
ser.start()

fvs.start()

serial_counter = 0
# Initialize start time - its to correlate frame with time stamp marker
t = 0
prev_frame_time = time.time_ns()
new_frame_time = 0

#store time and angle values from each source to plot
time_cam = []
time_imu = []
time_enc = []
angle_cam = []
angle_imu = []
angle_enc = []

#initialize plotting for 3 sensors
fig, ax = plt.subplots(1,1)
ax.set_xlim(0, 30)
ax.set_ylim(-180, 180)
plt.show(block=False)
(points_cam,) = ax.plot(time_cam, angle_cam, animated=True, label = 'Camera Angle')
plt.pause(0.1)


ax1 = ax
(points_imu,) = ax1.plot(time_imu, angle_imu, animated=True, label = 'IMU Angle')
ax.draw_artist(points_cam)


ax2 = ax
(points_enc,) = ax2.plot(time_enc, angle_enc, animated=True, label = 'Encoder Angle')
ax.draw_artist(points_cam)

legend = ax.legend()

background = fig.canvas.copy_from_bbox(fig.bbox)
fig.canvas.blit(fig.bbox)


while (True):
    if(ser.more()): #if more serial data available
        ard_data = ser.read()
        serial_counter += 1
        prev_frame_time = ser.sync_time
        print(ard_data)
        try:    
            ax_imu = float(ard_data[1])
            ay_imu = float(ard_data[2])
            
            gx_imu = float(ard_data[3])
            
            theta_enc = float(ard_data[4])
            time_ard = float(ard_data[0])/1e3
            
            angle_enc.append(theta_enc)
            time_enc.append(time_ard)
            points_enc.set_data(time_enc,angle_enc)
            
            theta_accel = (180/np.pi)*np.arctan2(ax_imu,ay_imu)
            theta_accel_filt = LPF.update(theta_accel) #this filters the accelerometer signal using the LPF defined
            theta_gyro = (180//np.pi)*Int.update(gx_imu) #this integrates gyro data
            theta_gyro_filt = HPF.update(theta_gyro) #this filters integrated gyro data using the HPF defined
            theta_est = theta_accel_filt + theta_gyro_filt #perform sensor fusion
            
            angle_imu.append(theta_est) #save fused data
    
            time_imu.append(time_ard)
            points_imu.set_data(time_imu,angle_imu)

            with open(filename_Proc_Ard + '.csv', 'a', newline='') as file: #this will save processed arduino data (time, encoder, fused)
                writer = csv.writer(file)
                writer.writerow([str(time_ard),str(theta_enc),str(theta_est)])
                
        except:
            time_imu.append(float('nan'))
            points_imu.set_data(time_imu,angle_imu)

        
    if(fvs.more() and serial_counter>0): #check if there are frames available
        # Store read frame; first parameter for frame presence boolean, second actual frame
        frame = fvs.read()
       
        # Convert frame to grayscale
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        # Set ArUco marker dictionary size - aruco.DICT_aXb_c : a and b refer to number of bits in marker (exclude boundary layer), and c is number of markers in dictionary
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        
        # Initialize detector parameters
        parameters = aruco.DetectorParameters_create()
	
        # Set detector threshold; default = 7 (constant subtracted from weighted mean); range of input = -255 to +255, see readme for adaptive thresholding
        parameters.adaptiveThreshConstant = 6

        # Initialize lists for corners and IDs, discard rejected corners
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # If IDs list is populated - pass condition is if a marker is detected successfully in a frame. if missing or out of frame, then it wont go into if condition
        if np.all(ids != None):
            #In the next two lines, extract the id and corner locations only of the Aruco specified (id = 0) (improves stability) 
            ids = ids[[np.where(ids==arucoID)[0][0]]]
            corners = [corners[np.where(ids==arucoID)[0][0]]]
            # Estimate pose of each marker and return values; 2nd parameter input (0.045, units meter) = marker length; 3rd output = array of object points of all the marker corners
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.045, cmatrix, dmatrix)

            # Iterate through recognized IDs
            #for i in range(0, ids.size): its between 0 to 1 because we only expect 1 marker to be detected
            for i in range(0, 1):
                # Draw ArUco axes for visualization
                aruco.drawAxis(frame, cmatrix, dmatrix, rvec[i], tvec[i], 0.1)
            
                # Extract rotation matrix from rotation vector
                rot = R.from_rotvec(rvec[i])
                #extract yaw
                yaw_cam = float(np.round(rot.as_euler('zyx', degrees=True)[0][0], 3))
                angle_cam.append(yaw_cam) 
                time_cam.append(float(t))
                
                if(serial_counter%3==0): #only plot every 3 iterations for faster rendering
                    fig.canvas.restore_region(background)
                    points_cam.set_data(time_cam,angle_cam)
                    ax.draw_artist(points_cam)
                    ax1.draw_artist(points_imu)
                    ax2.draw_artist(points_enc)
                    ax.draw_artist(legend)
                    fig.canvas.blit(fig.bbox)
                    fig.canvas.flush_events()
                
                    if(round(t%30)<0.03 and t>5):
                        low_lim = 30*round(t/30)
                        high_lim = low_lim + 30
                        ax.set_xlim(low_lim, high_lim)
                        ax.set_ylim(-180, 180)
                        ax1.set_xlim(low_lim, high_lim)
                        ax1.set_ylim(-180, 180)
                        
                # Write time, angle (roll, pitch, yaw) in degrees, and ID to CSV file; row then column for rotation matrix
                # by default the rotation vector is in the format zyx
                # it will not overwrite csv file with same name, it will only append the file. it uses the same file name as the video
                with open(filename + '.csv', 'a', newline='') as file:
                    writer = csv.writer(file)
                    #index rotation vector to be in line with desired coordinate frame (XYZ, forward, side, top)
                    #Time, Angle X, Angle Y, Angle Z, Aruco ID
                    writer.writerow([t,str(np.round(rot.as_euler('zyx', degrees=True)[0][1], 3)),str(np.round(rot.as_euler('zyx', degrees=True)[0][2], 3)),str(np.round(rot.as_euler('zyx', degrees=True)[0][0], 3)),ids[i][0]])
                    
                        
            # Draw squares around detected markers for visualization
            aruco.drawDetectedMarkers(frame, corners)

            # Initialize detected markers ID array for visualization
            strid = ''
        
            # Populate detected markers ID array for visualization
            for i in range(0, ids.size):
                strid += str(ids[i][0])+', '

            # Show IDs found for visualization
            cv.putText(frame, "ID: " + strid, (0,64), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv.LINE_AA)

        else:
            # Indicate lack of IDs for visualization
            cv.putText(frame, "No IDs", (0,64), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv.LINE_AA)
            n = len(angle_cam) #if NaN, repeat previous data point for plotting
            if(n==0):
                angle_cam = [0] #if marker is out of field of view initially
                time_cam = [0]
            angle_cam.append(angle_cam[n-1]) 
            time_cam.append(time_cam[n-1])
            # Write time and lack of angle and ID to CSV file
            with open(filename + '.csv', 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([t, "NaN", "NaN", "NaN", "NaN"])
   
        #    Update time increment with retrieved current position of the video
        new_frame_time = (time.time_ns() - prev_frame_time)/(1.0e9)
        t = new_frame_time
        # Show current frame
        cv.imshow('frame',frame)

        # Interrupt with esc if needed
        if cv.waitKey(1) & 0xff == 27: #Waits for 1ms for preseed key 27 (esc). To close the image press and hold the Esc key
            print('Output generation interrupted.')
            break
        

# Stop capture
fvs.stream.release()

plt.close(fig)

fvs.stop()

ser.stop()

# Close opened windows
cv.destroyAllWindows()

# Print output confirmation
print('Output file', filename + '.csv','generated.')


