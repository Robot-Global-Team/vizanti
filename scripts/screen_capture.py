#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from mss import mss
from PIL import Image as PILImage
import io

class ScreenCaptureNode:
    def __init__(self):
        rospy.init_node('screen_capture_node', anonymous=True)
        self.image_pub = rospy.Publisher('screen_capture/compressed', CompressedImage, queue_size=10)
        self.rate = rospy.Rate(1)  # 1Hz (1초에 한 번)
        self.sct = mss()

    def pil_to_compressed_ros_image(self, pil_image):
        ros_image = CompressedImage()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.format = "jpeg"
        
        # PIL 이미지를 JPEG로 압축
        img_buffer = io.BytesIO()
        pil_image.save(img_buffer, format='JPEG')
        ros_image.data = img_buffer.getvalue()
        
        return ros_image

    def capture_and_publish(self):
        while not rospy.is_shutdown():
            try:
                # 전체 화면 캡처
                screen = self.sct.grab(self.sct.monitors[0])
                
                # PIL Image로 변환
                img = PILImage.frombytes("RGB", screen.size, screen.bgra, "raw", "BGRX")
                
                # PIL 이미지를 CompressedImage 메시지로 변환
                ros_image = self.pil_to_compressed_ros_image(img)
                
                # ROS 토픽으로 발행
                self.image_pub.publish(ros_image)
                rospy.loginfo("Published compressed image to ROS topic")
            except Exception as e:
                rospy.logerr(f"Error publishing to ROS topic: {e}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ScreenCaptureNode()
        node.capture_and_publish()
    except rospy.ROSInterruptException:
        pass