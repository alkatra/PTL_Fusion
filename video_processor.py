import librosa
import numpy as np
import cv2
import torch
from moviepy.editor import VideoFileClip, AudioFileClip
from pydub import AudioSegment
from joblib import load
from sklearn.ensemble import RandomForestClassifier
import os
import time
import math
import rospy
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO 

# Initialize ROS node and publisher
rospy.init_node('video_processor', anonymous=True)
pub = rospy.Publisher('/traffic_light_state', Int32MultiArray, queue_size=10)
rate = rospy.Rate(4)  # Publish at 4Hz

class VideoProcessor:
    def __init__(self, video_path, fusion_model='models/fused_model.joblib', video_model='models/ptl_yolo_model.pt'):
        # Load models and set video path
        self.video_path = video_path
        self.fusion_model = load(fusion_model)
        self.video_model = YOLO(video_model)
        
    def process_video(self):
        # Process the video to extract features and predict traffic light state
        video = cv2.VideoCapture(self.video_path)
        fps = video.get(cv2.CAP_PROP_FPS)
        frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        resolution = (int(video.get(cv2.CAP_PROP_FRAME_WIDTH)), int(video.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        audio = AudioSegment.from_file(self.video_path)
        clip = audio[0:250]
        clip.export("temp_clip.mp3", format="mp3")
        clip_audio, sample_rate = librosa.load("temp_clip.mp3", sr=None)
        detections, size, sum_feature1, sum_feature2 = 0, 0, 0, 0
        
        for i in range(0, frame_count, 2):  # Process every other frame for efficiency
            video.set(cv2.CAP_PROP_POS_FRAMES, i)
            ret, frame = video.read()
            results = self.video_model.predict(source=frame, verbose=False, device=0, conf=0.01)
            extracted = self.extract_traffic_light_coords(results)
            if extracted is not None:
                detections += 1
                x1, y1, x2, y2, confidence = extracted
                size = (x2-x1) * (y2-y1)
                image = frame[y1:y2, x1:x2]
                frame_features = self.extract_image_features(image, confidence)
                sum_feature1 += frame_features[0]
                sum_feature2 += frame_features[1]

        # Calculate average vision features from detections
        vision_features = (sum_feature1/detections, sum_feature2/detections) if detections != 0 else (0, 0)
        audio_features = self.extract_audio_features(clip_audio, sample_rate)
        features = np.concatenate((vision_features, audio_features)).reshape(1, -1)
        prediction = self.fusion_model.predict(features)
        
        # Prepare and publish the ROS message
        fusion_predicted_class = "Red" if prediction == 0 else "Green"
        result = Int32MultiArray(data=[int(prediction == 1), size])
        rospy.loginfo(result)
        pub.publish(result)
        
        video.release()
        os.remove(self.video_path)  # Cleanup by removing the processed video
        
    def extract_audio_features(self, audio, sr, n_mfcc=24):
        # Extract audio features using MFCC
        mfccs = librosa.feature.mfcc(y=audio, sr=sr, n_mfcc=n_mfcc)
        return mfccs.mean(axis=1)        

    def extract_traffic_light_coords(self, results):
        # Extract coordinates of the most confident traffic light detection
        traffic_lights = (results[0].boxes.cls == 0.).nonzero(as_tuple=True)[0]
        if len(traffic_lights) == 0:
            return None
        confidence_values = results[0].boxes.conf[traffic_lights]
        most_confident_light, idx_traffic_light = torch.max(confidence_values, 0)
        box = results[idx_traffic_light].boxes.xyxy[idx_traffic_light]
        x1, y1, x2, y2 = map(int, box)
        return x1, y1, x2, y2, most_confident_light.item() * 100
    
    def extract_image_features(self, img, confidence):
        # Extract color features from the traffic light image area
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
        green_mask = cv2.inRange(hsv, (75, 50, 50), (100, 255, 255))
        red_count, green_count = np.sum(red_mask == 255), np.sum(green_mask == 255)
        if red_count + green_count == 0:
            return 0, 0
        return (red_count/(red_count+green_count)) * 100, (green_count/(red_count+green_count)) * 100
