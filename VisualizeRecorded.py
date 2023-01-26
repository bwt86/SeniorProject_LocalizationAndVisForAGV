"""
VisualizeRecorded.py
====================================
Visualized recorded data from a given file, running the file will visualize the recorded path.
"""
#import all needed libraries
import open3d as o3d
import numpy as np
from plyfile import PlyData, PlyElement
import time
import copy
#make sure main thread runs this so autodoc doesn't break
if __name__ == "__main__":
    #speed factor for slowing down translation (higher number = more slowed translations = slower visualized movement)
    speedFactor = 1000000
    #factor to downsample the main pointcloud by, smaller factor = less down sampled (so a factor of 2 gets rid of less points then a factor of 20)
    downsampleFactor = 2
    downsampleFactor = downsampleFactor * (1/100)
    #factor to convert feet to viewfinder meters
    convertFactor = .25
    #mult by 12 because data uses inches
    convertFactor = convertFactor * 12


def readFromFile(fileName):
    """Reads all data from a given for use in other functions

    :param fileName: name of file to read data from
    
    :return: Return all data from file as a single string (it does not process this data)
    :rtype: String
    
    """
    #open data file
    dataFile = open(fileName)
    #read data and close file
    data = dataFile.read()
    dataFile.close()
    #return raw data read from file
    return data
    
def getCoordData(fileName):
    """Gets all coordinate data from a given file

    :param fileName: name of file to read data from
    
    :return: Return coordinate data from file
    :rtype: List

    """
    #get raw coordinate data
    rawCoordData = readFromFile(fileName)
    #split data into lines
    lines = rawCoordData.split("\n")
    coordData = []
    #for each line
    for line in lines:
        coord = []
        #split data into values
        vals = line.split(",")
        #for all lines that contain coord data
        if len(vals) == 2:
            #convert each value to a float
            for i in vals:
                coord.append(float(i)) 
            #append float coord to coord data 
            coordData.append(copy.deepcopy(coord))
    #return visualized data
    return coordData

def genPath(data):
    """Generates path given coordinate data

    :param data: coordinate data to make path from

    :return: Return Path generated for coordinate data for visualization
    :rtype: List

    """
    path = []
    transVals = [0,0,0]
    last =[0,0,0]
    #for each coordinate
    for i in data:
        #for each value
        for j in range(len(i)):
            #calculate tranlate and set new last translation value to be current coordinate after calculation
            transVals[j] = (i[j]/convertFactor) - last[j]
            last[j] = i[j]/convertFactor
        #append translation to path    
        path.append(copy.deepcopy(transVals))
    #print(path)
    extendedPath = []

    # extend path by breaking down translation into a number of subtranslations based on the speed factor
    for i in path:
        for j in range(speedFactor + 1):
            extendedPath.append([i[0]*(j/speedFactor),i[1]*(j/speedFactor),0])
    #make the path the extended path
    path = extendedPath
            
    return path

#main routine should be executed by main thread        
if __name__ == '__main__':    

    #read pointcloud
    pcd = o3d.io.read_point_cloud('LivingRoom8.ply') 

    #coordinate frame that represents robot
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[0, 0, 0])

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
    #get path to render
    path = genPath(getCoordData('dataOut.txt'))
    #render points in path
    for i in range(len(path)):
        mesh_frame.translate(path[i])
        vis.update_geometry(mesh_frame)
        vis.poll_events()
        vis.update_renderer()
        #this stops visualizer from freezing/locking (not 100% sure why but the devs for Open3D recomend this to solve viewfinder locking so it works)
        if not vis.poll_events():
            break

    #run and destroy visualizer window (don't know if this code is really still needed)
    vis.run()
    vis.destroy_window()


