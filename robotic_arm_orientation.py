# Designed and Programmed by: Leif Ekstrom
# Last Updated: November 29th, 2020
#
# This program allows the user to simulate and plot the orienation of a robotic arm in 3D.
# You can create as many frames as you want. 
# Each frame can be of an x, y, z, or identity/prismatic rotation type. 
# The length of each robotic arm segment can also be specified, as can amount of degrees to rotate.

import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class HTM:
    # Used to create homogeneous transformation matrices which are relative to the previous robot frame.
    # This function assumes that each robotic arm segment is initially laying on the x axis
    #
    # frameList: list of all frames which currently exist, and to which this frame will be added
    # objName: string with the name of the object being created. This is used for print statements
    # matrixType: string that defines what type of matrix is being created. Takes lower or uppercase letters
    #             X = X (roll) axis transformation matrix
    #             Y = Y (pitch) axis transformation matrix
    #             Z = Z (yaw) axis transformation matrix
    #             I = identity rotatioin matrix or prismatic joint
    # segmentLength: the length of robotic arm segment 
    # theta: angle (in degrees) being rotated. Can be positive or negative. DO NOT use if matrixType is identity
    #-------------------------------------------------------------------------------------------------------------
    
    def __init__(self, objName, matrixType, segmentLength, theta=None):
        self.objName = objName
        self.matrixType = matrixType.upper()
        self.segL = segmentLength
        # ensure that theta value is converted to radians for calculation purposes
        if theta is not None:
            self.theta = math.radians(theta)
        else:
            self.theta = None
        
        if self.matrixType == 'X':
            self.mat = [   [1,                               0,                               0,                                 self.segL],
                           [0,  round(math.cos(self.theta), 5), round(-math.sin(self.theta), 5),                                         0],
                           [0,  round(math.sin(self.theta), 5),  round(math.cos(self.theta), 5),                                         0],
                           [0,                               0,                               0,                                         1]]
        elif self.matrixType == 'Y':
            self.mat = [   [ round(math.cos(self.theta), 5), 0,  round(math.sin(self.theta), 5),  self.segL*round(math.cos(self.theta), 5)],
                           [                              0, 1,                               0,                                         0],
                           [round(-math.sin(self.theta), 5), 0,  round(math.cos(self.theta), 5),  self.segL*round(math.sin(self.theta), 5)],
                           [                              0, 0,                               0,                                         1]]
        elif self.matrixType == 'Z':
            self.mat = [   [round(math.cos(self.theta), 5), round(-math.sin(self.theta), 5), 0,   self.segL*round(math.cos(self.theta), 5)],
                           [round(math.sin(self.theta), 5),  round(math.cos(self.theta), 5), 0,   self.segL*round(math.sin(self.theta), 5)],
                           [                             0,                               0, 1,                                          0],
                           [                             0,                               0, 0,                                          1]]
        elif self.matrixType == 'I' and self.theta is None:
            self.mat = [   [1, 0, 0, self.segL],
                           [0, 1, 0,         0],
                           [0, 0, 1,         0],
                           [0, 0, 0,         1]]
        # if the matrixType does not match a pre-defined type
        else:
            print('\nSomething went wrong in the definition of ' + self.objName)


    def print_mat(self):
        #prints the matrix in a way that is easier for humans to read
        print('Printing: ' + str(self.objName))
        print(*self.mat, sep="\n") 
        print('\n')
        
        
        
class CalculatedHTM:
    # Used to calculate homogeneous rotation matrices which are relative to frame 0.
    #
    # objName: string with the name of the object being created. This is used for print statements
    # r1: previous frame T_0x where x is the number of the robot frame which has been calculated up to
    # r2: the translation matrix for the frame which comes sequentially after frame T_0x
    #----------------------------------------------------------------------------------------------------
    def __init__(self, objName, r1, r2):
        self.objName = objName
        self.r1 = r1
        self.r2 = r2
        
        # calclate the resultant relative homogeneous translation matrix using dot product
        self.mat = np.dot(r1, r2)

    def print_mat(self):
        #prints the matrix in a form which is easier to read
        print('Printing: ' + str(self.objName))
        print(self.mat) 
        print('\n')



def graph_frames(R):
    # graphs the orientation and position of the robot frames relative to the world frame
    #
    # the thicker lines are the orientation of the world frame
    # the thinner lines are the orientation of frame R_0x
    #       red line = X axis
    #       blue line = Y axis
    #       green line = Z axis
    # orange speheres represent joints
    #-----------------------------------------------------------------------------------------
    
    # create the plot and define its parameters
    fig = plt.figure(num='Robotic Arm Orientation')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)

    # datasets will hold the coordinates required to draw the axes of each frame
    # this list stores the info that will be called by the plot function later
    # initialized with only the world frame
    datasets = []
    # stores the coordinates of joints
    xLoc, yLoc, zLoc = [], [], []
    
    # iterates through the list of frames 'R' and adds the coordinates required to draw each of their axes
    # appends to datasets
    for i in range(len(R)):
        jointSize = 9 # the size of joint markers on the plot
        segmentThickness = 2.5 # thickness of the robotic arm segments
        thickness = 1 # axis line thickness
        # line thickness of the world frame's axes
        if i == 0:
            thickness = 2
        # X axis 
        datasets.append({"x":[R[i].mat[0][3],(R[i].mat[0][0]+R[i].mat[0][3])],
                         "y":[R[i].mat[1][3],(R[i].mat[1][0]+R[i].mat[1][3])],
                         "z":[R[i].mat[2][3],(R[i].mat[2][0]+R[i].mat[2][3])],
                         "color": "red", "lw" : thickness})
        # Y axis 
        datasets.append({"x":[R[i].mat[0][3],(R[i].mat[0][1]+R[i].mat[0][3])],
                         "y":[R[i].mat[1][3],(R[i].mat[1][1]+R[i].mat[1][3])],
                         "z":[R[i].mat[2][3],(R[i].mat[2][1]+R[i].mat[2][3])],
                         "color": "blue", "lw" : thickness})
        # Z axis 
        datasets.append({"x":[R[i].mat[0][3],(R[i].mat[0][2]+R[i].mat[0][3])],
                         "y":[R[i].mat[1][3],(R[i].mat[1][2]+R[i].mat[1][3])],
                         "z":[R[i].mat[2][3],(R[i].mat[2][2]+R[i].mat[2][3])],
                         "color": "green", "lw" : thickness})
        # store joint coordinates
        xLoc.append(R[i].mat[0][3])
        yLoc.append(R[i].mat[1][3])
        zLoc.append(R[i].mat[2][3])
        # draw lines (robotic arm segments) connecting frames
        if i > 0:
            datasets.append({"x":[R[i-1].mat[0][3],(R[i].mat[0][3])],
                             "y":[R[i-1].mat[1][3],(R[i].mat[1][3])],
                             "z":[R[i-1].mat[2][3],(R[i].mat[2][3])],
                             "color": "black", "lw" : segmentThickness})

    for dataset in datasets:
        ax.plot(dataset["x"], dataset["y"], dataset["z"], color=dataset["color"], linewidth=dataset["lw"])
        ax.scatter(xLoc,yLoc,zLoc,color='orange',s=jointSize)

    fig.suptitle('Robotic Arm Orientation', fontsize=14)
    ax.set_xlabel('X Direction')
    ax.set_ylabel('Y Direction')
    ax.set_zlabel('Z Direction')
    plt.show()
    
    
    
def printFrameList(frameList):
    # prints all frames in the list
    frameStr = 'Frame List: '
    for i in frameList:
        frameStr += i.objName + ', '
    print(frameStr[:-2] + '\n')   



def main():
    # Create homogeneous transformation matrices for joints
    # from a given axis of rotation, segment length, and rotation amount (in degrees)---------------------------------------
    frameList = [] # list which holds all frames
    R_00 = HTM('R_00','i',0) # R_00 is the world frame
    frameList.append(R_00)
    
    T_01 = HTM('T_01','z',5,150)
    frameList.append(T_01) # all frames after R_01 must be calculated before they can be added to frameList
    T_12 = HTM('T_12','y',4,100)
    T_23 = HTM('T_23','z',3,-90)
    T_34 = HTM('T_34','z',3,-60)
    T_45 = HTM('T_34','x',1.5,90)

    # Calculate rotation matrices relative to the world frame---------------------------------------------------------------
    T_02 = CalculatedHTM('T_02', T_01.mat, T_12.mat)
    frameList.append(T_02)
    T_03 = CalculatedHTM('T_03', T_02.mat, T_23.mat)
    frameList.append(T_03)
    T_04 = CalculatedHTM('T_04', T_03.mat, T_34.mat)
    frameList.append(T_04)
    T_05 = CalculatedHTM('T_05', T_04.mat, T_45.mat)
    frameList.append(T_05)
    printFrameList(frameList)

    # Display the robotic arm-----------------------------------------------------------------------------------------------
    graph_frames(frameList)



if __name__=="__main__": 
    main() 