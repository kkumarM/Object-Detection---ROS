#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from apple_detection_pkg.msg import detected, detectionData

from ultralytics import YOLO
import cv2

class AppleDetection():

    def __init__(self, input_topic, output_topic, des_size, model_path, model1_path):
        self.size_categories = {
    'small': (5000, 10000, (0, 0, 255)),      # Red for small apples
    'medium': (10000, 15000, (0, 255, 0)),   # Green for medium apples
    'large': (15000, 50000, (255, 0, 0))     # Blue for large apples
}
        self.model = YOLO(model_path)
        self.model1 = YOLO(model1_path)
        self.subscriber = rospy.Subscriber(input_topic, Image, self.imageCallback)
        self.publisher = rospy.Publisher(output_topic+"/annotated_image", Image, queue_size=1)
        self.data_publisher = rospy.Publisher(output_topic+"/prediction_data", detectionData, queue_size=1)
        self.bridge = CvBridge()
        self.detected_img = None
        self.detected_data = None
        self.des_size = des_size
        self.verbose = log

    def imageCallback(self, img):
        if self.model != None:
            msg = self.bridge.imgmsg_to_cv2(img, "bgr8")
            self.detected_img = self.processing(msg)
            self.imagePublisher()

    def imagePublisher(self):
        if log: rospy.loginfo("Publishing Result.")
        self.publisher.publish(self.bridge.cv2_to_imgmsg(self.detected_img))
        self.data_publisher.publish(self.detected_data)

    def processing(self,msg):
        self.detected_data = detectionData()
        apple_areas = []
        results = self.model.predict(msg, verbose=False)
        image = msg
        result = results[0]
        detected_coordinates=[]
        for box in result.boxes:
            cords = box.xyxy[0].tolist()
            class_id = box.cls[0].item()
            conf = box.conf[0].item()
            
            l = "Apple" if class_id == 0 else "Apple"
            if conf > 0.35:

                x1,y1,x2,y2 = int(cords[0]),int(cords[1]),int(cords[2]),int(cords[3])
                cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
                detected_coordinates.append((x1,y1,x2-x1,y2-y1))
                cv2.putText(image,l+"-"+str(round(float(conf),2)),(x1,y1),cv2.FONT_HERSHEY_SIMPLEX,1,	(0,0,255),2)

        
        results = self.model1.predict(msg, verbose=False)
        result = results[0]
        for box in result.boxes:
            cords = box.xyxy[0].tolist()
            class_id = box.cls[0].item()
            conf = box.conf[0].item()
            # print(cords, end = " , ")
            # print(class_id, end = " , ")
            # print(conf)
            _detected = detected()
            _detected.score = conf
            _detected.label= "Apple" if class_id == 0 else "Apple"
            _detected.xyxy = cords
            self.detected_data.data.append(_detected)
            
            if class_id == 0:
                l = "Raw"
            if class_id == 1:
                l='Ripe'
            elif class_id == 2:
                l="Rot"

            if conf > 0.50:

                x1,y1,x2,y2 = int(cords[0]),int(cords[1]),int(cords[2]),int(cords[3])
                
                cv2.rectangle(image,(x1,y1),(x2,y2),(0,0,0),2)
                cv2.putText(image,l+"-"+str(round(float(conf),2)),(x1+200,y1),cv2.FONT_HERSHEY_SIMPLEX,1,	(0,140,255),2)
        for x, y, width, height in detected_coordinates:
            # Calculate the area of the detected apple
            area = width * height
            apple_areas.append((area, (x, y, width, height)))

        # Sort the detected apples by area (largest first)
        sorted_apples = sorted(apple_areas, key=lambda x: x[0], reverse=True)

        # Sort the detected apples by size
        classified_apples = []
        for area, (x, y, width, height) in sorted_apples:
            size = None
            for category, (min_area, max_area,color) in self.size_categories.items():
                if min_area <= area <= max_area:
                    size = category
                    c=color
                    break
            classified_apples.append((size,color, (x, y, width, height)))
        for (size,color, (x, y, width, height)) in classified_apples:
            # Calculate the center coordinates of the detected apple
            cX = x + width // 2
            cY = y + height // 2

            # Draw a center dot
            cv2.circle(image, (cX, cY), 5, color, -1)

            # Display the size classification next to the apple
            cv2.putText(image, size, (cX - 50, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
        return image

if __name__ == "__main__":
    rospy.init_node("apple_detection_node")

    usecase = rospy.get_param("/usecase", "apple_detection")
    input_topic = rospy.get_param("/camera_input_topic", "/front_cam/camera/image")
    output_topic = rospy.get_param("/output_topic", "/apple_detection/detected")
    image_size = rospy.get_param("/image_size", [640, 480])
    model_path = rospy.get_param("/model_path", '../weights/detection.pt')
    model1_path = rospy.get_param("/model1_path", '../weights/classification.pt')
    log = rospy.get_param("/log_to_screen", False)
    
    rospy.loginfo("Starting "+ usecase + " Node!")
    
    cam = AppleDetection(input_topic, output_topic, image_size, model_path, model1_path)
    rospy.spin()
