#!/usr/bin/python3
from ultralytics import YOLO
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from robot_vision.msg import Vision

# ROS node that deals with the extraction of information resulting from the application of the Yolov8 framework 
# to the images received from the camera publication topic ("/husky_model/husky/camera")

#height = 240
#width = 320
pub = None
# The CvBridge is an object that converts between OpenCV Images and ROS Image messages.
bridge = CvBridge()
# Core object for Yolov8 usage
model = YOLO("yolov8s.pt")

# This ROS node sends on the "vision" topic information related to the position 
# on the image seen by the camera of the human target (if found)
target_found = False
center_x = 160.
center_y = 120.
target_x = 0.
target_y = 0.

# method for applying the Yolov8 machine learning model on the image extracted from the camera for humans' detection
def box_extractor(image):
    global target_found, center_x, center_y, target_x, target_y

    img = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    
    # img -> image in OpenCV format
    # classes=0 -> specialize the detection on human beings
    result = model.predict(img, classes=0, verbose=False, show=False)[0] # we take the first human being found (predict result element of index 0)
    
    # extraction of the position of the rectangle displayed by Yolov8 when a human being is detected
    box = result.boxes
    if ((box.cls.nelement() != 0) and ("person" == result.names[int(box.cls[0])])):
        coords = box.xyxy[0]
        target_found = True

        #extraction of the coordinates of the rectangle center
        target_x = (coords[0] + coords[2]) / 2
        target_y = (coords[1] + coords[3]) / 2
    else:
        target_found = False
    
    pub.publish(Vision(target_found, center_x, center_y, target_x, target_y))

# ROS node initialization method 
# register and listen on the "/husky_model/husky/camera" topic to receive images (Image ROS msg) from the camera
def v_talker():
    rospy.init_node("v_talker")
    
    global pub
    pub = rospy.Publisher('vision', Vision, queue_size=10)

    rospy.Subscriber("/husky_model/husky/camera", Image, box_extractor, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        v_talker()
    except rospy.ROSInterruptException:
        pass