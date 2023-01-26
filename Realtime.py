"""
Realtime.py
====================================
Runs full system integration in real time
"""
#import all needed libraries
import threading
import open3d as o3d
import numpy as np
from plyfile import PlyData, PlyElement
import time
import sys
import copy
import cv2
from pyzbar.pyzbar import decode

#Speed factor to slow down translation speed (higher = slower translation)
speedFactor = 10
#factor to convert feet to viewfinder meters
convertFactor = 10
#mult by 12 because data uses inches
convertFactor = convertFactor * 12
#factor to downsample the main pointcloud by, smaller factor = less down sampled (so a factor of 2 gets rid of less points then a factor of 20)
downsampleFactor = 2
downsampleFactor = downsampleFactor * (1/100)

def readCalFromFile(fileName):
    """Reads calibration data from a given file

    :param fileName: Name of file to read coord data from

    :return: Returns data read from calibration file
    :rtype: List

    """
    #Opens data file, reads, then closesit
    dataFile = open(fileName)
    data = dataFile.read()
    dataFile.close()
    #split data into list separated by commas
    data = data.split(',')
    dataFin =[]
    #make all the data into floats and put it into final data structure for returning
    for i in data:
        dataFin.append(float(i))
    return dataFin

        
            
#Gets distance
def getDistance(sideLen, focalLen, camWidth):
    """Gets distance to a code given the required knowns in the system of real side length, calibrated focal length of the camera, and the width of the code in the viewfinder

    :param sideLen: Real world size of qr code in inches (from one edge of code to the other)
    :param focalLen: Calibrated focal length of the camera (unit is pixels)
    :param camWidth: Width of the Qr code in the viewfinder in pixels

    :return: Returns the distance to a QR code in inches
    :rtype: float

    """
    return (sideLen*focalLen)/camWidth

##Multithreading setup
#get threading condition for locking
c = threading.Condition()
# get flag for locking and sequencing
flag = 0 
#create datastuctures and variables to be used
locData = []
tempData = []
count = 0;
changed = False;
#path datastructure to store robot translations
path = []
#how many translations should be captured before killing threads (if a limit is desired)
maxCount = 10

class CamThread(threading.Thread):
    """Class for creating a camera thread for capturing localization data

    :param threading.Thread: thread class from python threading library to use for creating thread

    """
    def __init__(self, name, camNo):
        """Initalized the CamThread class instance and starts thread

        :param self: The reference to this class instance itself
        :param name: Name of thread
        :param camNo: Camera number for the camera this thread represents

        """
        threading.Thread.__init__(self)
        self.name = name
        self.camNo = camNo
        self.cap = cv2.VideoCapture(self.camNo)
        

    def capFrame(self):
        """Captures frames and attempts to get distance from these images

        :param self: The reference to this class instance itself

        :return: Return distance generated from capturing frames
        :rtype: float

        """
        global focal
        #set capture size for viewfinder
        self.cap.set(3,640)
        self.cap.set(4,480)
        #time.sleep(.01)
        #read in qr code for use with this camera (uses nameing convention Qr + camNo + .png for finding files)
        img = cv2.imread(f'Qr{self.camNo}.png')
        #while the camera needs to capture frames to get a distance
        needCap = True
        while needCap:
            #read in a new frame
            success, img = self.cap.read()
            #for each code found
            for i in decode(img):
                print(i.rect)
                print(i.data.decode('utf-8'))
                # generate pts array and reshape for getting distance
                pts = np.array([i.polygon], np.int32)
                pts = pts.reshape((-1,1,2))
                pts2 = i.rect
                #variable created for stretch goal features involving decoding code data we didn't get to implement
                #decodeTxt = i.data.decode('utf-8')
                #if a real distance is generated we dont need to capture and we should return distance
                if getDistance(7,focal[self.camNo],pts2[3]) > 0:
                    needCap = False
                    return getDistance(7,focal[self.camNo],pts2[3])
            
    def run(self):
        """Routine to run after initalization of the thread. \n
        This routine waits for resources to become available and when they are the thread then:\n

        1. Changes the flag to the correct value for program operation
        2. Captures frames to get distance
        3. Appends data to the temporary data structure 
        4. Looks to see if the theres enough distance data for coordinate
            * If so:
                1. Append coordinate to path data structure
                2. Clear Temporary data structure
            * Otherwise: 
                1. Pass on the data to the next capture thread to get more location data for a full coordinate

        :param self: The reference to this class instance itself 

        """
        global count
        global flag
        global maxCount
        while True:
            #get resource lock
            c.acquire()
            #check flag is correct indicating resource availability
            if flag == self.camNo:
                #set flag for next resource
                flag = self.camNo + 1
                #get a distance from the camera and append it to our temporary coordinate holding list
                tempData.append(self.capFrame())
                #time.sleep(0.01)
                #notify all threads about resource lock status (not sure if this is still needed but it fixed some issues earlier in development)
                c.notify_all()
                #if we have a 2D coord in the temp data holding structure
                if len(tempData) == 2:
                    #append a deepcopy of this coordinate (appending directly not desirable because of memory refrence issues)
                    locData.append(copy.deepcopy(tempData))
                    #clear temp data structure
                    tempData.clear()
                    #use this as a mechanism to stop the thread if we only want to observe x amount of movement changes
                    count += 1  
            #release resource lock
            c.release()
            # if count is being used once a certain amount of movements are captured it will kill the thread and release camera resource
            if count >= maxCount:
                self.cap.release()
                sys.exit()



class PathGenThread(threading.Thread):
    """Class for creating a path generation thread

    :param threading.Thread: thread class from python threading library to use for creating thread

    """
    def __init__(self):
        """initializes class for path generation and starts thread\n

        This thread also initialized 2 lists for storing data using in the thread's run function

        :param self: The reference to this class instance itself 

        """
        self.transVals = [0,0,0]
        self.last = [0,0]
        threading.Thread.__init__(self)
    
    def run(self):
        """Generates path given coordinate data.\n
        This thread Aims to generate and extend a path of translations for the robot for which the visualizer by doing the following:\n
            1. See if theres a coordinate available to be made into a translation
                * Assuming a coordinate is available:
                    1. Figure out the change from the last coordinate and convert this change from real life units to rendered units using conversion factor
                    2. Set the last coordinate to be current coordinate
                    3. Artifically extend path into number of sub points based on speed factor set by user
                * Otherwise: don't do anything

        :param self: The reference to this class instance itself

        """
        global path
        global flag
        global locData
        global count
        global maxCount
        while True:
            #get resource lock
            c.acquire()
            #check flag is correct
            if flag == 2:
                #set flag for next function
                flag = 3
                #if theres data in location data (there should be)
                if len(locData) > 0:
                    #pop off the first entry in the location data queue
                    i = locData.pop(0)
                    #for each coordinate
                    for j in range(len(i)):
                        #calculate tranlate and set new last translation value to be current coordinate after calculation
                        self.transVals[j] = (i[j]/convertFactor) - self.last[j]
                        #print(f'trans = {(i[j]/convertFactor)} - {self.last[j]} = {(i[j]/convertFactor) - self.last[j]}')
                        self.last[j] = i[j]/convertFactor
                        #print(f'last = {self.last}')
                    #append translation to path    
                    path.append(copy.deepcopy(self.transVals))
                    
                    #create temp data structure for extending path
                    extendedPath = []
                    
                    # extend path by breaking down translation into a number of subtranslations based on the speed factor
                    for i in path:
                        for j in range(speedFactor + 1):
                            extendedPath.append([i[0]*(j/speedFactor),i[1]*(j/speedFactor),0])
                    #make the path the extended path
                    path = extendedPath
            #release resource lock
            c.release()
            # if count is being used once a certain amount of movements are captured it will kill the thread
            if count >= maxCount:
                sys.exit()
                        
class VisThread(threading.Thread):
    """Class for creating a visualization thread to manipulate position of robot in visualization

    :param threading.Thread: thread class from python threading library to use for creating thread

    """
    def __init__(self):
        """initializes class for visualization manipulations and starts thread\n

        :param self: The reference to this class instance itself 

        """
        threading.Thread.__init__(self)     


    def run(self):
        """Manipulates visualizer data to show translation of robot.\n

        Works with main thread to get all translations rendered by:\n
        1. Checking if path has translations that can be rendered\n
            **Assuming there translations to be rendered, while there is tranlations to be rendered:**
                
                1. Assuming the variable changed is false (meaning the visualizer is rendered up to date) render a translation to the mesh
                    
                    * Note: this does not update the visualizer, it just changes the mesh objects state

                2. Change the state of changed to true

                3. Main thread sees that the chaned variable has changed and updates visualizer by rerendering with updated geometry
                    
                    * Note:  this must be done in main thread because of how Open3D works

                4. Main thread changes changed variable to false restarting the cycle to sub-step 1\n
            
            **Otherwise: do nothing**

        2. Reset flag value to restart realtime process

        :param self: The reference to this class instance itself git 

        """
        global vis
        global mesh_frame
        global path
        global flag
        global changed
        global convertFactor
        global maxCount
        while True:
            #aquire resource lock
            c.acquire()
            #check flag calue
            if flag == 3:
                #print(path)
                #while theres stuff in the path data structure
                while len(path) > 0:
                    #assuming the system doesn't see any current changes in the visualization to render
                    if(changed == False):
                        #get a single translation value from path and edit mesh frame state
                        singleTranslation = path.pop(0)
                        mesh_frame.translate(singleTranslation)
                        # tell the system the state of the geometry has change so main thread can update render
                        changed = True
                flag = 0;
                #print(path)
            #release resource lock
            c.release()
            # if count is being used once a certain amount of movements are captured it will kill the thread
            if count >= maxCount:
                sys.exit()

#makes sure only the main thread runs the main routine (for document generation purposes)
if __name__ == "__main__":
    #Get camera calibration data
    focal = readCalFromFile('cal.txt')

    #create camera threads for cams 0 and 1 and start a path generation thread
    a = CamThread("CamA", 0)
    b = CamThread("CamB", 1)
    e = PathGenThread()
    #create vizualizer 
    vis = o3d.visualization.Visualizer()
    vis.create_window('Robot',width = 1920, height = 1080)
    pcd = o3d.io.read_point_cloud('LivingRoom8.ply') 
    #create mesh frame that represents robot
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.6, origin=[0, 0, 0])
    #downsample point cloud
    finPCD = pcd.voxel_down_sample(voxel_size=downsampleFactor)
    #add all geometry
    vis.add_geometry(finPCD)
    vis.add_geometry(mesh_frame)
    #get viewfinder controls and set it to look at the correct spot with correct camera setting
    ctr = vis.get_view_control()
    ctr.set_lookat([ -0.14692800000000006, -0.25343749999999998, 1.9695 ])
    ctr.set_front([ 0.061652851348087713, -0.37022503897342068, -0.9268939240483669 ])
    ctr.set_up([ 0.022672879033815659, -0.92790143034758643, 0.37213556147621696 ])
    ctr.set_zoom(0.7)
    # render geometry
    vis.update_geometry(mesh_frame)
    vis.poll_events()
    vis.update_renderer()
    #make visualization thread
    d = VisThread()
    #start all threads
    d.start()
    e.start()
    b.start()
    a.start()

    #double check that this is being run by main thread (probably not necessary but sanity check)
    if __name__ == "__main__":
        while True:
            #if the geometry has changed update the render then set changed value to false so next robot move can be rendered
            if changed:
                vis.update_geometry(mesh_frame)
                vis.poll_events()
                vis.update_renderer()
                changed = False
            #this stops visualizer from freezing/locking (not 100% sure why but the devs for Open3D recomend this to solve viewfinder locking so it works)
            if not vis.poll_events():
                break
            

    #join all threads to wait till completion 
    a.join()
    b.join()
    e.join()
    d.join()


