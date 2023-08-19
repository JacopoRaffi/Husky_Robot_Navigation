#!/usr/bin/python3
from ultralytics import YOLO
import rospy
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from robot_vision.msg import Vision
#height = 240
#width = 320
pub = None
bridge = CvBridge()
model = YOLO("yolov8s.pt")
target_found = False
center_x = 160.
center_y = 120.
target_x = 0.
target_y = 0.

def box_extractor(image):
    #a = time.time()*1000
    global target_found, center_x, center_y, target_x, target_y
    img = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    result = model.predict(img, classes=0, verbose=False, show=False)[0] 
    
    box = result.boxes
    if ((box.cls.nelement() != 0) and ("person" == result.names[int(box.cls[0])])):
        coords = box.xyxy[0]
        target_found = True
        target_x = (coords[0] + coords[2]) / 2
        target_y = (coords[1] + coords[3]) / 2
    else:
        target_found = False
    
    pub.publish(Vision(target_found, center_x, center_y, target_x, target_y))
    #b = time.time()*1000

    #print(b-a)


def v_talker():
    rospy.init_node("v_talker")
    
    global pub
    pub = rospy.Publisher('vision', Vision, queue_size=10)

    rospy.Subscriber("/husky_model/husky/camera", Image, box_extractor, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        v_talker()
    except rospy.ROSInterruptException:
        pass