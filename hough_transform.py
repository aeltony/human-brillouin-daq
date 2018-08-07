import cv2
import numpy as np
import random
import math
import skvideo.io as skv

start_point = None
end_point = None
expected_center = None

def return_pupil_diameter(event,x,y,flags,param):
    global start_point, end_point
    if event == cv2.EVENT_LBUTTONDOWN:
        start_point = (x,y)
    elif event == cv2.EVENT_LBUTTONUP:
        end_point = (x,y)

def return_expected_center(event,x,y,flags,param):
    global expected_center
    if event == cv2.EVENT_LBUTTONDOWN:
        expected_center = (x,y)

def set_expected_radius(frame):
    cv2.namedWindow("set expected radius")
    cv2.setMouseCallback("set expected radius",return_pupil_diameter)

    while True:
        cv2.imshow("set expected radius",frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("x"):
            break

    return int(math.sqrt((start_point[0] - end_point[0])**2 + (start_point[1] - end_point[1])**2)/2)

def set_expected_center(frame):
    cv2.namedWindow("set expected center")
    cv2.setMouseCallback("set expected center",return_expected_center)

    while True:
        cv2.imshow("set expected center",frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("x"):
            break

    return expected_center


# what we are trying to minimize which takes into account difference in radius and center
def objective_function(center,radius,expected_center,expected_radius):
    center_diff = math.sqrt((center[0] - expected_center[0])**2 + (center[1] - expected_center[1])**2)
    radius_diff = abs(radius - expected_radius)

    return center_diff + 5*radius_diff

        
        
"""
## Parameters for cv2.HoughCircles() ##
image: 8-bit, single channel image. If working with a color image, convert to grayscale first.
method: Defines the method to detect circles in images. Currently, the only implemented method is cv2.HOUGH_GRADIENT, which corresponds to the Yuen et al. paper.
dp: This parameter is the inverse ratio of the accumulator resolution to the image resolution (see Yuen et al. for more details). Essentially, the larger the dp gets, the smaller the accumulator array gets.
minDist: Minimum distance between the center (x, y) coordinates of detected circles. If the minDist is too small, multiple circles in the same neighborhood as the original may be (falsely) detected. If the minDist is too large, then some circles may not be detected at all.
param1: Gradient value used to handle edge detection in the Yuen et al. method.
param2: Accumulator threshold value for the cv2.HOUGH_GRADIENT method. The smaller the threshold is, the more circles will be detected (including false circles). The larger the threshold is, the more circles will potentially be returned.
minRadius: Minimum size of the radius (in pixels).
maxRadius: Maximum size of the radius (in pixels).
"""
def detect_pupil_frame(frame,medianBlur,dp,minDist,param1,param2,radius_range,expected_radius,coordinates,ROI_center=None):
    
    if frame is None: 
        return

    dim = frame.shape
    frame_bgr = frame.copy()

    if ROI_center is not None:
        crop_size = 2*expected_radius
        min_y, max_y, min_x, max_x = max(0,ROI_center[1]-crop_size), min(dim[0],ROI_center[1]+crop_size), max(0,ROI_center[0]-crop_size), min(dim[1],ROI_center[0]+crop_size)
        cropped_frame = frame_bgr[min_y:max_y, min_x:max_x]
    else:
        cropped_frame = frame_bgr


    frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)

    frame = cv2.medianBlur(frame,medianBlur) #required for Hough transform

    circles = cv2.HoughCircles(frame,cv2.HOUGH_GRADIENT,dp,minDist,param1=param1,param2=param2,minRadius=expected_radius-radius_range,maxRadius=expected_radius+radius_range)

    min_objective = float('inf')
    min_circle_center = None
    min_circle_radius = None
    
    # multiple circles are fine as long as we only draw in the one with smallest radius differential with expected
    if circles is not None: 
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            #also show other detected circles
            #cv2.circle(frame_bgr,(i[0],i[1]),i[2],(0,0,255),1)
            #cv2.circle(frame_bgr,(i[0],i[1]),2,(0,0,255),2)

            objective = abs(expected_radius - i[2])

            if objective < min_objective:
                min_objective = objective
                min_circle_center = (i[0],i[1])
                min_circle_radius = i[2]

        if coordinates:
            x,y = min_circle_center
            partition_size = 100
            partitions = min_circle_radius/partition_size + 1
            partition_radius = partitions*partition_size
            for p in range(1,partitions+1):
                cv2.circle(cropped_frame,min_circle_center,p*partition_size,(0,255,0),1)

            cv2.line(cropped_frame,(x,min(y+partition_radius,dim[0])),(x,max(y-partition_radius,0)),(0,255,0),1)
            cv2.line(cropped_frame,(min(x+partition_radius,dim[1]),y),(max(x-partition_radius,0),y),(0,255,0),1)

    else:
        pass 
        #print "No circles detected!"

    #control 
    cv2.line(cropped_frame,(0,0),(0,cropped_frame.shape[0]),(255,255,255),5) # vertical line
    cv2.line(cropped_frame,(0,0),(cropped_frame.shape[1],0),(255,255,255),5) # horizontal line

    if ROI_center is not None and min_circle_center is not None:
        min_circle_center = (min_circle_center[0]+min_x,min_circle_center[1]+min_y)

    return (frame_bgr,min_circle_center,min_circle_radius)


def detect_pupil_video(path,radius_range=None):
    cap = cv2.VideoCapture(path)

    # ask user for expected radius #
    ret,frame = cap.read()
    print "Frame resolution: ",frame.shape

    #expected_radius = set_expected_radius(frame)
    #print "Expected radius: ",expected_radius

    # detect circles with expected radius #
    while cap.isOpened():
        ret,frame = cap.read()

        if frame is None:
            return

        frame_bgr,min_circle_center,min_circle_radius = detect_pupil_frame(frame,radius_range=radius_range)

        if frame_bgr is None:
            break

        cv2.imshow('detected circles',frame_bgr)
        
        key = cv2.waitKey(100) & 0xFF
        if key == ord("x"):
            break


if __name__ == "__main__":
    detect_pupil_video("pupil_videos/AmiraOD010_CCD.avi",30)
    #most stable reading: AmiraOD011_CCD.avi, radius_range=15, median_blur=25
    #stable during movement example: AmiraOD016_CCD.avi
    #lots of blinking example: Amira0S003_CCD.avi
    #another setting that works: dp=3, param2=700