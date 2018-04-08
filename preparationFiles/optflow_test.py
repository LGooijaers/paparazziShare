# -*- coding: utf-8 -*-
"""
Created on Sat Mar 10 22:40:48 2018

@author: Rick
"""



#%% initialize parameters
useOrb = True
useGoodFeat = False
useGrid = False
useFast = False
useContours = False
useCells = False
cellrows = 2
cellcols = 5
numCells = cellrows*cellcols
numFeaturepoints = 100
plotting = True
numberofSkips = 1
livePlotting = False

import cv2
import numpy as np
import time
from farnback import *
import matplotlib.pyplot as plt
import os


video = "testdrone.mp4"
video2 = "walk.flv"
video3 = "slow.flv"
cap  =cv2.VideoCapture(video)
framerate = cap.get(5)

#%%set size of images
frameH = int(cap.get(4))
frameW = int(cap.get(3))
#%%initialize subframes for feature detection
subW = int(frameW/cellcols)
subH = int(frameH/cellrows)
onesW = np.ones(subW)
onesH = np.ones(subH)


#%% initialize blocks
subH = frameH/cellrows;
subW = frameW/cellcols;

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
   
    return blocks.astype(np.uint8);








blocks = init_blocks(subH,subW,frameH,frameW,numCells,cellrows,cellcols)
totalP = []
totalError = []
totalTtC = []
RunningTime = []
grid=[]
dist = 50  
for i in range(frameW):
    for j in range(frameH):
        if ((i% dist == 0) and (j%dist == 0)):
            grid.append(1)
        else:
            grid.append(0)
grid = np.float32(np.array(grid)).reshape(frameW,frameH)
if (useGrid==False):
    grid=None


#%% function for detection in cells
if (useCells):
    N = numFeaturepoints/numCells
    def fastdetect(img):
        p0 = np.array([]).reshape(0,1,2).astype(np.uint8)
        for i in range(len(blocks)):
            subimg = img * blocks[i]
            kp = FeatDetect.detect(subimg,mask = grid)
            subp = np.float32(np.array([o.pt for o in kp])).reshape(len(kp),1,2)
            p0 = np.vstack((p0,subp))
        return p0
else:
    N = numFeaturepoints
    def fastdetect(img):
        
        kp = FeatDetect.detect(img,mask = grid)
        p0 = np.float32(np.array([o.pt for o in kp])).reshape(len(kp),1,2)
        return p0
        
#%% Function for linearizing optical flow field
#def(p0,p1):
def linearize_field(points_old, points_new,RANSAC=False, n_iterations=10, error_threshold=10.0):
    flow_vectors= (points_new-points_old)
    n_points = points_old.shape[0];
    sample_size = 3; # minimal sample size is 3
    
    if(n_points >= sample_size):
        
            
            
       if(not RANSAC):
            
            # *****************************************
            # TODO: investigate this estimation method:
            # *****************************************
            
            # estimate a linear flow field for horizontal and vertical flow separately:
            # make a big matrix A with elements [x,y,1]
            A = np.concatenate((points_old, np.ones([points_old.shape[0], 1])), axis=1);
           
            # Moore-Penrose pseudo-inverse:
            # https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
            pseudo_inverse_A = np.linalg.pinv(A);
            
            # target = horizontal flow:
            u_vector = flow_vectors[:,0];
            # solve the linear system:
            pu = np.dot(pseudo_inverse_A, u_vector);
            # calculate how good the fit is:
            errs_u = np.abs(np.dot(A, pu) - u_vector);
            
            # target = vertical flow:
            v_vector = flow_vectors[:,1];
            pv = np.dot(pseudo_inverse_A, v_vector);
            errs_v = np.abs(np.dot(A, pv) - v_vector);
            err = (np.mean(errs_u) + np.mean(errs_v)) / 2.0;
            
       else:
            # This is a RANSAC method to better deal with outliers
            # matrices and vectors for the big system:
            A = np.concatenate((points_old, np.ones([points_old.shape[0], 1])), axis=1);
            u_vector = flow_vectors[:,0];
            v_vector = flow_vectors[:,1];
            
            # solve many small systems, calculating the errors:
            errors = np.zeros([n_iterations, 2]);
            pu = np.zeros([n_iterations, 3])
            pv = np.zeros([n_iterations, 3])
            for it in range(n_iterations):
                inds = np.random.choice(range(n_points), size=sample_size, replace=False);
                AA = np.concatenate((points_old[inds,:], np.ones([sample_size, 1])), axis=1);
                pseudo_inverse_AA = np.linalg.pinv(AA);
                # horizontal flow:
                u_vector_small = flow_vectors[inds, 0];
                # pu[it, :] = np.linalg.solve(AA, UU);
                pu[it,:] = np.dot(pseudo_inverse_AA, u_vector_small);
                errs = np.abs(np.dot(A, pu[it,:]) - u_vector);
                errs[errs > error_threshold] = error_threshold;
                errors[it, 0] = np.mean(errs);
                # vertical flow:
                v_vector_small = flow_vectors[inds, 0];
                # pv[it, :] = np.linalg.solve(AA, VV);
                pv[it, :] = np.dot(pseudo_inverse_AA, v_vector_small);
                errs = np.abs(np.dot(A, pv[it,:]) - v_vector);
                errs[errs > error_threshold] = error_threshold;
                errors[it, 1] = np.mean(errs);
            
            # take the minimal error
            errors = np.mean(errors, axis=1);
            ind = np.argmin(errors);
            err = errors[ind];
            pu = pu[ind, :];
            pv = pv[ind, :];
    else:
        # not enough samples to make a linear fit:
        pu = np.asarray([1.0]*3);
        pv = np.asarray([1.0]*3);
        err = error_threshold;
    
        
    return pu, pv, err;

def divergence(pu,pv):
    FoE = np.asarray([0.0]*2);
    horizontal_motion = -pu[2]; # 0.0;
    vertical_motion = -pv[2]; #0.0;
   # theoretically correct, but numerically not so stable:
    FoE[0] = -pu[2]/pu[0]; #0.0;
    FoE[1] = -pv[2]/pv[1]; #0.0;
    divergence = (pu[0]+pv[1]) / 2.0; 
    Ttc= 1/divergence# 0.0;
    return FoE,divergence,horizontal_motion, vertical_motion
#def countP(p):
    
#%% params for ShiTomasi corner detection
feature_params = dict( maxCorners = 300,
                       qualityLevel = 0.1,
                       minDistance = 10,
                       blockSize = 7 )
                  
                       
#%% Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

#%% Create colors for drawing
red = np.array([0,0,255])
green = np.array([0,255,0])

# Take first frame and convert 2gray
ret, old_frame = cap.read()
#old_frame = cv2.resize(old_frame,(frameW,frameH))
#old_frame = cv2.cvtColor(old_frame, cv2.COLOR_YUV2RGB_)
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)




#%% detection of keypoints

if ret == True:
    
    if(useOrb):
        FeatDetect = cv2.ORB_create(nfeatures = N,scoreType=cv2.ORB_FAST_SCORE)
        p0 = fastdetect(old_gray)
    if(useGoodFeat):
        p0 = cv2.goodFeaturesToTrack(old_gray, mask = grid, **feature_params)
    
    if(useFast):
        FeatDetect = cv2.FastFeatureDetector_create(type=cv2.FastFeatureDetector_TYPE_9_16)
        p0 = fastdetect(old_gray)
    
        

    totalFeatures = len(p0)
    
    # Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)
    
    
#%% main program loop: Loop over video tracking optical flow
    starttime = time.time()
    skipframes = 0
    plt.figure
    plt.ion()
    while(1):
        
        ret,frame = cap.read()
        if(not ret):
            break
        frame = cv2.resize(frame,(frameW,frameH))
        if skipframes % numberofSkips ==0:
            
            
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            if p0.shape[0]<10:
                break
            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            
            #calc contours
            if(useContours):
                ret,thresh = cv2.threshold(frame_gray,127,255,0)        
                frame2, contour,what = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #       rgbflow = optical_flow(old_frame,frame,frameH,frameW)
           
           # Select good points for drawing and bookkeeping, removing points that could not be tracked
            good_new = p1[st==1]
            good_old = p0[st==1]
           
           #estimate movement
            ttcL = []
            x_movL = []
            y_movL = []
            FoeL = []
            errL = []
            
            for cells in range(cellcols):
               pu,pv,err = linearize_field(p0[(p0[:,:,0]>cells*subW) * (p0[:,:,0]<(cells+1)*subW)], p1[(p0[:,:,0]>cells*subW) * (p0[:,:,0]<(cells+1)*subW)])
               Foe, ttc,x_mov,y_mov = divergence(pu,pv)
               ttcL.append(ttc)
               FoeL.append(Foe)
               x_movL.append(x_mov)
               y_movL.append(y_mov)
               errL.append(err)
        
            totalError.append(err)
            totalTtC.append(ttcL)
            RunningTime.append(time.time()-starttime) 
    #        pu,pv,err = linearize_field(p0.reshape(p0.shape[0],2),p1.reshape(p0.shape[0],2))
    #        Foe, ttc,x_mov,y_mov = divergence(pu,pv)
            # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel()
            c,d = old.ravel()
            mask = cv2.line(mask, (a,b),(c,d), red, 2)
            frame = cv2.circle(frame,(a,b),5,red,-1)
            
            
              
            
            
#        mask = np.zeros_like(old_frame)
#        for cell in range(cellcols):
#            img = cv2.putText(frame,str(int(ttcL[cell])),(cell*subW+subW/2,frameH/2),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0))
#            img = cv2.putText(img,"x "+str(int(x_movL[cell])),(cell*subW+subW/2,frameH-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0))
#            img = cv2.putText(img,"y "+str(int(y_movL[cell])),(cell*subW+subW/2,frameH/2-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0))
#            img = cv2.putText(img,"FoE "+str(FoeL[cell]),(cell*subW+10,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0))
#            img = cv2.putText(img,"Err "+str(errL[cell]),(cell*subW+10,100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0))
        if(useContours):
            cv2.drawContours(frame,contour,-1,(0,255,0),3)
        
        for i in range(cellrows-1):
            cellLines = cv2.line(frame,(0,(i+1)*subH),(frameW,subH*(i+1)),green,1)
            
        for i in range(cellcols-1):
            cellLines = cv2.line(frame,((i+1)*subW,0),((i+1)*subW,frameH),green,1)
        
        
        img = cv2.add(frame,mask)
        cv2.imshow('frame',img)
#        cv2.imshow('mask',mask)
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
       
#%%If number of point drop beow threshold update points
       # Now update the previous frame and previous points
        if len(p1)<=totalFeatures*0.8:
            
            old_gray=frame_gray.copy()
            
            if(useGoodFeat):
                p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
            
            else:
                p0 = fastdetect(old_gray)
            
            
            totalFeatures = len(p0)
            mask = np.zeros_like(old_frame) # keep this line if you would like to remove all previously drawn flows
        else :
            old_gray = frame_gray.copy()
            p0 = good_new.reshape(-1,1,2)
            
        totalP.append(len(p0))
        
        
        if(livePlotting and skipframes % 100 == 0): 
#            for i in range(cellcols):
#                plt.subplot(5,1,i+1)
            plt.plot(RunningTime[-1],ttcL[2],'r+')
            plt.pause(0.00000000000001)
        skipframes +=1
cv2.destroyAllWindows()
cap.release()
print ("np.mean(totalP)")
totalTtC = np.array(totalTtC)
totalTtC[totalTtC >100] = 100
totalTtC[totalTtC <0] = 0
#totalTtC[totalTtC <5] = 0
#totalTtC[totalTtC >5] = 1
if plotting == True:
    plt.figure()
    plt.plot(RunningTime,totalError)
    plt.xlabel('Time')
    plt.ylabel('Error')
    plt.figure()
    for i in range(cellcols):
        plt.subplot(5,1,i+1)
    
        plt.plot(RunningTime,totalTtC[:,i])
    plt.xlabel('Time')
    plt.ylabel('Time To Contact')
#print(len(RunningTime))


