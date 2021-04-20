import sys, time

# numpy and scipy
import numpy as np

import cv2
import matplotlib.pyplot as plt
#from scipy.ndimage import filters



# Ros libraries
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
# Ros Messages
from sensor_msgs.msg import CompressedImage


bridge = CvBridge()
cntr =0
save_image=0
def grey(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
def gauss(image):
    return cv2.GaussianBlur(image, (3, 3), 0)
def canny(image, low_thresh, high_thresh):
    edges = cv2.Canny(image,low_thresh,high_thresh)
    
    return edges


def region(image):
    height, width = image.shape
    triangle = np.array([
                       [(0, height), (320, 240), (width, height)]
                       ])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, triangle, 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def average(image, lines):
    left = []
    right = []
    for line in lines:
        print(line)
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_int = parameters[1]
        if slope < 0:
            left.append((slope, y_int))
        else:
            right.append((slope, y_int))
    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])


def make_points(image, average): 
    slope, y_int = average 
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1-y_int) // slope)
    x2 = int((y2-y_int) // slope)
    return np.array([x1, y1, x2, y2])

def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    return lines_image











def draw_point(u,v,img):
    center_coordinates = (int(u),int(v))
 
    # Radius of circle
    radius = 2
    
    # Blue color in BGR
    color = (255, 0, 0)
    
    # Line thickness of 2 px
    thickness = 3
    
    # Using cv2.circle() method
    # Draw a circle with blue line borders of thickness of 2 px
    img = cv2.circle(img, center_coordinates, radius, color, thickness)

    return img


def scene_geometry(img):
    camera_height=1
    Z=1
    umax = img.shape[0]
    vmax= img.shape[1]
    fx=320
    fy=240
    cx=320
    cy=240
    X=0.5
    Y=0

    u=(fx*X)/Z+cx
    v=(fy*Y)/Z+cy
    print("v,u", v,u)
    print("img[v,u]",img[int(v),int(u)])
    #img[int(v),int(u)] = 252
    img = draw_point(u,v,img)
    

    return img






def callback(ros_data):
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    global cntr
    global save_image

    #----------Receive 

   
    try:
        image_np = bridge.compressed_imgmsg_to_cv2(ros_data,"bgr8")
        copy = np.copy(image_np)
        #cv2.imshow('Image',image_np)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #grey1 = grey(copy)
        gaus = gauss(copy)
        edges = canny(gaus,50,300)
        
        isolated = region(edges)
        rho=50                 #The resolution of the parameter r in pixels
        theta=np.pi/180         #The resolution of the parameter theta in rad
        threshold=90            #The minimum number of intersections (in Hough plane) to "*detect*" a line.i.e. The minimum number of points (on cartesian) that lie on same line with slope equal to point of intersection (Hough space). Remember, number of votes (intersections) depend upon number of points on the line. So it represents the minimum length of line that should be detected.

        lines = cv2.HoughLinesP(isolated, rho, theta, threshold, np.array([]), minLineLength=40, maxLineGap=5) 
        cntr=cntr+1
        print("lines \n",lines)
        if lines is not None:
            print("Size",len(lines))
            if(len(lines)>2):
                '''
                averaged_lines = average(copy, lines)
                black_lines = display_lines(copy, averaged_lines)
                lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
                '''
                if(save_image):

                    cv2.imwrite("/PATH_TO_OUT_DIRECTORY/"+str(cntr)+"_Image_isolated.jpg", isolated)
                    cv2.imwrite("/PATH_TO_OUT_DIRECTORY/"+str(cntr)+"_Image_lanes.jpg", lanes)
            
        else:
            print("No lines detected")
        

            


        #----------Publish
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', edges)[1]).tostring()
        # Publish new image
        image_pub.publish(msg)
    except CvBridgeError, e:
        print(e)

    '''
    #----------Process
    copy = np.copy(image_np)
    #grey1 = grey(copy)
    gaus = gauss(copy)
    edges = canny(gaus, 50,300)'''
    


    


def main_ros():
    subscriber = rospy.Subscriber("/camera/image_raw/compressed",CompressedImage, callback,  queue_size = 10)
    rospy.init_node('line_detector', anonymous=True)

    image_pub = rospy.Publisher("/line_detect/image_raw/compressed",CompressedImage)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

    
    
    
if __name__ == '__main__':
    #main_ros(sys.argv)
    #main_ros()
    
    subscriber = rospy.Subscriber("/camera_rect/image_raw/compressed",CompressedImage, callback,  queue_size = 10)
    rospy.init_node('line_detector', anonymous=True)

    image_pub = rospy.Publisher("/line_detect/image_raw/compressed",CompressedImage)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

