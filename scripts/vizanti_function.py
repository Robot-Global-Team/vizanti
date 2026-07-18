#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import io
import os
import requests
import json

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path  # 예시로 Path 메시지를 사용
from sensor_msgs.msg import CompressedImage
import unicodedata
from PIL import Image as PILImage
import subprocess
import sys
import time
try:
    from mss import mss

except:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "mss"])
    print("✅ 'mss' 설치 완료!")
    print("❌ 설치 중 오류 발생:", e)
    from mss import mss

os.environ["DISPLAY"] = ":0"

class ScreenCaptureNode:
    def __init__(self):
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
        try:
            # 전체 화면 캡처
            screen = self.sct.grab(self.sct.monitors[0])
            
            # PIL Image로 변환
            img = PILImage.frombytes("RGB", screen.size, screen.bgra, "raw", "BGRX")
            
            # PIL 이미지를 CompressedImage 메시지로 변환
            ros_image = self.pil_to_compressed_ros_image(img)
            
            # ROS 토픽으로 발행
            self.image_pub.publish(ros_image)
            #rospy.loginfo("Published compressed image to ROS topic")
        except Exception as e:
            rospy.logerr(f"Error publishing to ROS topic: {e}")
            
         

class TaskInfoVisualizer:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('task_info_listener', anonymous=True)
        self.tables = self.package_data()
        self.path_data = None
        self.cnt = 0

        self.screen_capture = ScreenCaptureNode()

        # MarkerArray 퍼블리셔 설정
        self.marker_pub = rospy.Publisher('package_table_array', MarkerArray, queue_size=10)


        # 토픽 구독 설정
        self.subscriber = rospy.Subscriber("/sirbot1/state_machine/task_info", String, self.callback)
        self.global_path = rospy.Subscriber("/sirbot1/smooth_path", Path, self.path_callback)

        self.path_visulization = rospy.Publisher("/global_path_visualization", Path, queue_size=10)



    def package_data(self):
        url = "http://localhost:51231/iface/package"
        try:
            response = requests.get(url)
            
            # 요청 성공 시
            if response.status_code == 200:
                data = response.json()
            else:
                print(f"⚠️ 요청 실패 - 상태 코드: {response.status_code}")
            
        except requests.exceptions.RequestException as e:
            print(f"🚨 요청 중 오류 발생: {e}")
        # "table" 항목만 추출하여 딕셔너리로 저장
        table_dict = {
            item["name"]: {"x": item["x"], "y": item["y"]}
            for item in data.values()
            if any(attr in item.get("attribute", []) for attr in ["table", "start"])
        }

        # 출력
        print(table_dict)
        return table_dict

    def callback(self, data):
        #print("!")
        self.publish_markers()
        if self.cnt % 10 ==0:
            self.screen_capture.capture_and_publish()
        self.cnt +=1
        
        if self.path_data is not None:
             self.path_visulization.publish(self.path_data)

    def path_callback(self, data):
        rospy.loginfo("Received path message")
        # 처음 받은 path 데이터를 저장
        self.path_data = data

    def clean_text(self, text):
        try:
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="strict")
            else:
                text = str(text)  # str로 강제 변환

            # 유니코드 정규화
            normalized = unicodedata.normalize("NFC", text)

            # UTF-8 encode/decode 테스트
            encoded = normalized.encode("utf-8", errors="strict")
            decoded = encoded.decode("utf-8", errors="strict")

            return decoded
        except Exception as e:
            rospy.logwarn(f"❌ 유니코드 오류로 마커 텍스트 제거됨: {repr(text)} → {e}")
            return ""

    
    def contains_korean(self,text):
        return any('\uAC00' <= char <= '\uD7A3' for char in text)

    def publish_markers(self):
        marker_array = MarkerArray()
        
        for idx, (name, pos) in enumerate(self.tables.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "table_circle"
            marker.id = idx
            marker.type = Marker.TEXT_VIEW_FACING if not self.contains_korean(name) else Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = 0.3
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            if marker.type == Marker.TEXT_VIEW_FACING:
                marker.scale.z = 0.35  # 텍스트 크기
                marker.text = name
            else:
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.05

            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)



    def spin(self):
        # 콜백 함수가 종료되지 않도록 대기
        rospy.spin()

if __name__ == '__main__':
    visualizer = TaskInfoVisualizer()
    visualizer.spin()