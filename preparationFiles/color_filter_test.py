
#This is a test of using a colorfilter in combination with a grid to find the location of obstacles
#This script only looks for orange obstacles.







#%%
import cv2
import numpy as np
import time
from farnback import *
import matplotlib.pyplot as plt
import os

plotting = False
useCells = True
cellrows = 2
cellcols = 5
numCells = cellrows*cellcols
red = np.array([0,0,255])
green = np.array([0,255,0])
white = np.array([255,255,255])


video = "testdrone.mp4"
cap  =cv2.VideoCapture(video)
framerate = cap.get(5)

#%%set size of images
frameH = int(cap.get(4)/4)
frameW = int(cap.get(3)/4)
subW = int(frameW/cellcols)
subH = int(frameH/cellrows)

#%%initialize subframes for feature detection
subW = int(frameW/cellcols)
subH = int(frameH/cellrows)
onesW = np.ones(subW)
onesH = np.ones(subH)




#%% function for creating arrays for each cell
def init_blocks(subH,subW,frameH,frameW,numCells,cellrows,cellcols):
    blocks= np.zeros([numCells,frameH,frameW]);
    activeCell = 0
    for i in range(cellcols):
      
        for j in range(cellrows):
          
          for k in range(frameH):
              
              for l in range(frameW):
                  if ((k>=j*subH) and (k<subH*(j+1)) and (l>=i*subW) and (l<subW*(i+1))):
                        blocks[activeCell][k][l]=1;
                  else:
                        blocks[activeCell][k][l]=0
          activeCell +=1
    squareArray = []     
    for i in range(cellcols):
        for j in range(cellrows):
#            squareArray.append(tuple([j*subH,i*subW]))
#            squareArray.append(tuple([(1+j)*subH,(1+i)*subW]))
            squareArray.append(tuple([i*subW,j*subH]))
            squareArray.append(tuple([(1+i)*subW,(1+j)*subH]))
    return blocks.astype(np.uint8), blocks[0].sum(),(squareArray)



#%% function for finding obstacle
def check_cell_for_color():
    mask_block = mask_range * blocks
    obstacles = (mask_block>0).sum(axis=2).sum(axis=1) > maxones * 0.05
    return obstacles

#%% function for drawing white lines for the cell structure
def draw_lines():
    for i in range(cellrows-1):
        cellLines = cv2.line(frame,(0,(i+1)*subH),(frameW,subH*(i+1)),white,1)
            
    for i in range(cellcols-1):
        cellLines = cv2.line(frame,((i+1)*subW,0),((i+1)*subW,frameH),white,1)

#%% function for creating square grid
def square_grid():
    squares = np.zeros((frameH,frameW,3), np.uint8)
    for i in range(numCells):
        if obstacles[i]:
            squares = cv2.rectangle(squares,squareArray[2*i],squareArray[2*i+1],green,thickness=cv2.FILLED)
        else:
            squares = cv2.rectangle(squares,squareArray[2*i],squareArray[2*i+1],red,thickness=cv2.FILLED)
    return squares


blocks,maxones,squareArray = init_blocks(subH,subW,frameH,frameW,numCells,cellrows,cellcols)

ret,frame = cap.read()

while(ret):
    frame = cv2.resize(frame,(frameW,frameH))
#    YUV_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2YUV)

    lower_color = np.array([0, 53, 134])
    upper_color = np.array([255, 121, 249])

    mask_range = cv2.inRange(frame, lower_color, upper_color)
    obstacles = check_cell_for_color()
#    res  = cv2.bitwise_and(frame, frame, mask = blocks[0])
    squares = square_grid()
    
#%% draw lines for images    
    draw_lines()
    

    cv2.imshow('frame', frame)
#    cv2.imshow('mask', mask)
#    cv2.imshow('res', res)
    cv2.imshow('output',squares)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    ret, frame = cap.read()
cv2.destroyAllWindows()
cap.release()