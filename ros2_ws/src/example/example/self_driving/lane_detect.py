#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/11
# @author:aiden
# lane detection for autonomous driving
import os
import cv2
import math
import queue
import threading
import numpy as np
import sdk.common as common
from cv_bridge import CvBridge

bridge = CvBridge() # ROS 이미지 변환 객체 생성

# LAB 색공간 기준값 (yaml파일 불러오기, /home/ubuntu/software/lab_tool/lab_config.yaml)
lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
'''
lab:
  Mono: # 모노 카메라용
    black:
      max:
      - 89
      - 255
      - 255
      min:
      - 0
      - 0
      - 0
    blue:
      max:
      - 255
      - 150
      - 106
      min:
      - 0
      - 0
      - 0
    dark_green:
      max:
      - 255
      - 255
      - 255
      min:
      - 0
      - 0
      - 0
    green:
      max:
      - 255
      - 110
      - 255
      min:
      - 47
      - 0
      - 135
    red:
      max:
      - 112
      - 186
      - 165
      min:
      - 85
      - 145
      - 147
    white:
      max:
      - 255
      - 255
      - 255
      min:
      - 193
      - 0
      - 0
  Stereo:
    black:
      max:
      - 90
      - 255
      - 255
      min:
      - 0
      - 0
      - 0
    blue:
      max:
      - 255
      - 255
      - 110
      min:
      - 0
      - 0
      - 0
    dark_green:
      max:
      - 128
      - 127
      - 130
      min:
      - 0
      - 0
      - 0
    green:
      max:
      - 255
      - 104
      - 255
      min:
      - 47
      - 0
      - 135
    red:
      max:
      - 255
      - 255
      - 255
      min:
      - 0
      - 165
      - 104
    white:
      max:
      - 255
      - 255
      - 255
      min:
      - 193
      - 0
      - 0
    yellow:
      max:
      - 255
      - 255
      - 255
      min:
      - 135
      - 129
      - 132
size:
  height:
  - 480
  width:
  - 640
'''

 # 차선 인식 클래스
class LaneDetector(object):
    def __init__(self, color): # 초기 값
        # lane color
        self.target_color = color # 감지할 차선 = 입력 color
        # ROI for lane detection
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera': # 카메라 종류가 ascamera인 경우
            
            # ROI 구간은 아래와 같다
            '''
            y=0 ──────────────────────────────── (영상 위쪽)
            │
            │   ┌───────────────────────────┐
            │   │                           │
            │   │        ROI #3             │  (248 ~ 270, weight=0.1)
            │   │                           │
            │   └───────────────────────────┘
            │
            │   ┌───────────────────────────┐
            │   │                           │
            │   │        ROI #2             │  (292 ~ 315, weight=0.2)
            │   │                           │
            │   └───────────────────────────┘
            │
            │   ┌───────────────────────────┐
            │   │                           │
            │   │        ROI #1             │  (338 ~ 360, weight=0.7)
            │   │                           │
            │   └───────────────────────────┘
            │
            y=480 ─────────────────────────────── (영상 아래쪽)
            '''
            self.rois = ((338, 360, 0, 320, 0.7), (292, 315, 0, 320, 0.2), (248, 270, 0, 320, 0.1))
        else: # ascamera가 아니라면

            '''
            y=0 ─────────────────────────────── (맨 위)
            │
            │
            │
            │
            │
            │   ROI #3 (330~480, weight=0.1)
            │   ┌───────────────────────┐
            │   │                       │
            │   │     ROI #2            │ (390~480, weight=0.2)
            │   │   ┌───────────────────┐
            │   │   │                   │
            │   │   │  ROI #1           │ (450~480, weight=0.7)
            │   │   │                   │
            │   │   └───────────────────┘
            │   │                       │
            │   └───────────────────────┘
            │
            y=480 ────────────────────────────── (맨 아래)

            '''
            self.rois = ((450, 480, 0, 320, 0.7), (390, 480, 0, 320, 0.2), (330, 480, 0, 320, 0.1))
        self.weight_sum = 1.0 # ROI 가중치 초기화

    # 신규 ROI 설정 시 사용
    def set_roi(self, roi):
        self.rois = roi # roi 값 재지정

    @staticmethod # 정적 메서드 -> 클래스와 무관하게 독립적으로 쓰임

    # 가장 큰 면적을 가진 컨투어(윤곽선)을 반환, 최소면적 = 100
    def get_area_max_contour(contours, threshold=100):
        '''
        obtain the contour corresponding to the maximum area
        :param contours:
        :param threshold:
        :return:
        '''
        # cv2.countourArea() : 면적 계산 (픽셀 단위 크기)
        # math.fabs() : 음수를 절대값 처리
        # map(lamda c:... , countours): countours 리스트 요소들의 면적 계산
        # zip(): 원래 컨투어와 면적을 짝지어 튜플로 묶음 ex) (contour1, 120.5), ...
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))

        # filter() : threshold 보다 큰 값만 남기기, 이터레이터로 반환하므로 튜플로 변환
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0: # 컨투어 값이 있다면
            max_c_a = max(contour_area, key=lambda c_a: c_a[1]) # 컨투어 면적 중 가장 큰 값 반환
            return max_c_a # 반환 값: (가장 큰 contour, 그 면적)
        return None # 아무것도 없다면 None
    
    # 수평 가상선 추가 (가까운 차선의 y좌표)
    def add_horizontal_line(self, image):
        #   |____  --->   |————   ---> ——
        h, w = image.shape[:2] # 이미지 크기 높이 구하기
        roi_w_min = int(w/2) # 너비 시작 (중간)
        roi_w_max = w # 너비 끝 (오른쪽)
        roi_h_min = 0 # 높이 시작 (아래)
        roi_h_max = h # 높이 끝 (위)

        # 위 ROI 부분만 짤라냄 (화면기준 오른쪽 화면)
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]  # crop the right half
        flip_binary = cv2.flip(roi, 0)  # flip upside down # 이미지 뒤집기 (OpenCV좌표계는 y=0이 최상단)

        # cv2.minMaxLoc() : 이미지에서 최소/최대 값과 위치 반환
        # min_val(픽셀 값), max_val(픽셀 값), min_loc(좌표), max_loc(좌표)
        # [-1][1] : maa_loc애서 y좌표값
        # 위쪽부터 탐색하며 가장 밝은 점을 탐색, 가장 가까운 쪽을 찾기 위해 뒤집기
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]  # extract the coordinates of the top-left point with a value of 255

        return h - max_y # 원래 y좌표로 반환
    '''
    원본 좌표계 (y=0 위, y=h 아래)
    0 ───────────────── (화면 위)
    ⋮
    430 ─────────────── (차선 위치)
    ⋮
    480 ─────────────── (화면 아래)

    뒤집힌 좌표계 (flip 후)
    0 ─────────────── (원래는 아래쪽)
    ⋮
    50 ────────────── (flip에서 찾은 max_y)
    ⋮
    480 ───────────── (원래는 위쪽)

    → h - max_y = 480 - 50 = 430
    '''

    # 수직 가상선 추가
    def add_vertical_line_far(self, image):
        h, w = image.shape[:2] # 이미지 높이, 너비 불러오기
        roi_w_min = int(w/8) # 이미지 1/8 지점
        roi_w_max = int(w/2) # 이미지 1/2 지점
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max] # 이미지 좌측 부분 ROI 설정

        # 상하좌우 동시 반전
        flip_binary = cv2.flip(roi, -1)  # flip the image horizontally and vertically
        #cv2.imshow('1', flip_binary)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ret)
        # minVal：the minimum value
        # maxVal：the maximum value
        # minLoc：the location of the minimum value
        # maxLoc：the location of the maximum value
        # the order of traversal is: first rows, then columns, with rows from left to right and columns from top to bottom
        
        # 가장 밝은 좌표를 구하기 (x, y)
        # min_val(픽셀 값), max_val(픽셀 값), min_loc(좌표), max_loc(좌표)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # extract the coordinates of the top-left point with a value of 255
        y_center = y_0 + 55 # 조금 더 아래를 y_center로 잡음
        roi = flip_binary[y_center:, :] # ROI 재설정 y최솟값 0 -> y_center
        (x_1, y_1) = cv2.minMaxLoc(roi)[-1] # 가장 밝은 좌표를 구하기 #2
        
        # 원래 좌표계로 변환 (down_p)
        down_p = (roi_w_max - x_1, roi_h_max - (y_1 + y_center))
        
        y_center = y_0 + 65 # 조금 더 아래를 y_center로 잡음
        roi = flip_binary[y_center:, :] # ROI 재설정 y최솟값 0 -> y_center
        (x_2, y_2) = cv2.minMaxLoc(roi)[-1] # 가장 밝은 좌표를 구하기 #3

        # 원래 좌표계로 변환 (up_p)
        up_p = (roi_w_max - x_2, roi_h_max - (y_2 + y_center))

        up_point = (0, 0) # 포인트 생성
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0: # up_p와 dp_p 값이 둘다 같지 않다면

            # ((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) --> 기울기 m
            # up_p[1] - down_p[1] = m * (up_p[0] - down_p[0])
            # up_p[0] = (up_p[1] - down_p[1])/m + down_p[0]
            # y=0 일 때 (최상단) up_p x값 , 0
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)

            # y=h일 때 (최하단) up_p x값, h
            down_point = (int((h - down_p[1])/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), h)
        
        return up_point, down_point
    '''
    y=0  ──────────● up_point( x_at_y0, 0 )
      \       
       \      ← 직선을 영상 위(y=0)와 아래(y=h)까지 연장
        \
         ● down_p( x_d, y_d )
          \
           \
    y=h  ───────● down_point( x_at_yh, h )
    '''

    # 보조 수직 가상선 (가장 가까운 영역에서 세로 보조선 잡기)
    def add_vertical_line_near(self, image):
        # ——|         |——        |
        #   |   --->  |     --->
        h, w = image.shape[:2]
        roi_w_min = 0
        roi_w_max = int(w/2)
        roi_h_min = int(h/2)
        roi_h_max = h # 좌측 하단 (전체화면의 1/4)를 지정
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max] # ROI로 잘라냄

        # 좌우+상하 동시반전, minMaxLoc이 좌상단부터 스캔하므로 가까운 차선을 뽑아내는 트릭
        flip_binary = cv2.flip(roi, -1)  # flip the image horizontally and vertically
        #cv2.imshow('1', flip_binary)
        
        # 가장 밝은(225)의 값을 가진 좌표 구하기
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # extract the coordinates of the top-left point with a value of 255
        
        # 원래 좌표계로 환산
        down_p = (roi_w_max - x_0, roi_h_max - y_0)

        # 반전되지 않은 이미지에서 가장 밝은 값을 가진 좌표 구하기
        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]

        # y1과 y0의 중간지점
        y_center = int((roi_h_max - roi_h_min - y_1 + y_0)/2)
        roi = flip_binary[y_center:, :] # 반전 이미지 y_min을 y_center로 설정
        (x, y) = cv2.minMaxLoc(roi)[-1] # 좌표 구하기
        up_p = (roi_w_max - x, roi_h_max - (y + y_center)) # 원래 좌표계로 반환

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:

            # y=0 일 때 (최상단) up_p x값, 0
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            
            # down_p 그대로 사용
            down_point = down_p

        return up_point, down_point, y_center
    '''
    전체 프레임 (640x480 가정)
y=0  ───────────────────────────────
│
│               [오른쪽 절반은 사용 안 함]
│
│  ┌──────────── 왼쪽 절반 (x=0 ~ 320) ────────────┐
│  │                                              │
│  │          (하단 영역: y=240 ~ 480)              │
│  │                                               │
│  │    up_p •                                     │  ← 두 번째 샘플
│  │         \                                     │
│  │          \                                    │
│  │           • down_p                            │  ← 첫 번째(하단) 샘플
│  │            \                                  │
│  │             \                                 │  ← up_point(y=0)까지 연장
│  └───────────────────────────────────────────────┘
│
y=480 ─────────────────────────────────────────────

    '''

    # 차선 색깔을 필터링 하여 이진이미지 만들기
    def get_binary(self, image):
        # recognize color through LAB space
        
        # 이미지 RGB이미지를 LAB으로 변환
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # convert RGB to LAB

        # 3x3커널로 가우시안 블러 처리(흐림)
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # Gaussian blur denoising

        # cv2.inRange : 특정 범위(min ~ max)안에 있는 픽셀을 255(흰색), 아니면 0(검정)
        # 색깔 범위만 남기고 나머지는 전부 버림
        # YAML 파일 안에서 읽은 색상 min, max 값
        # 특정 색만 흰색, 아니면 검은색으로 mask
        mask = cv2.inRange(img_blur, tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max']))  # 二值化

        # Erode(침식) 흰색 영역 가장자리를 줄여서 작은 점 잡음 제거
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # erode
        
        # Dilate(팽창) 침식으로 줄어든 영역을 다시 확대
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # dilate

        '''
        [원본 RGB] → [LAB 변환] → [블러로 노이즈 완화]
            ↓
        [색 범위 inRange] → 흰색(차선) vs 검정(배경)
            ↓
        [Erode] → 작은 잡음 제거
            ↓
        [Dilate] → 진짜 차선 강화
            ↓
        [최종 Binary 이미지 반환]

        '''
        return dilated

    # 이진 마스크 이미지 ROI별로 차선 중심 추정 -> 전체 중심 계산, 조향 각도 산출
    def __call__(self, image, result_image):
        # extract the center point based on the proportion
        centroid_sum = 0 # 가중치 초기화
        h, w = image.shape[:2] # 이미지 크기 : 높이, 너비
        max_center_x = -1 # 오른쪽 최대 값 초기화
        center_x = [] # ROI 별 중심 x값 리스트
        for roi in self.rois: # self.rois 값 순회
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # crop ROI # ROI 좌표로 잘라냄
            
            # ROI 흰색영역 외곽선들을 리스트로 얻음
            # cv2.RETR_EXTERNAL: 가장 바깥쪽 컨투어만 찾음 (안쪽 작은 것 무시).
            # cv2.CHAIN_APPROX_TC89_L1: 점들을 단순화해서 효율적으로 표현.
            # 외곽선 좌표들로 반환
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # find contours
            
            # 30픽셀보다 작은건 버리고 가장 넓은 컨투어 선택
            max_contour_area = self.get_area_max_contour(contours, 30)  # obtain the contour with the largest area
            if max_contour_area is not None: # 가장 넓은 컨투어가 있다면
                
                # 컨투어를 감싸는 최소 외접사각형
                rect = cv2.minAreaRect(max_contour_area[0])  # the minimum bounding rectangle
                
                # 사각형 네 꼭짓점 (ROI 내 좌표계)
                box = np.intp(cv2.boxPoints(rect))  # four corners
                for j in range(4): # y에 ROI 오프셋 더해 전역 좌표로 변환
                    box[j, 1] = box[j, 1] + roi[0]

                # 그릴 이미지, 윤곽선, 몇번째 윤곽선?:-1(전부), 색상(하늘색), 선 두께(2픽셀)
                cv2.drawContours(result_image, [box], -1, (255, 255, 0), 2)  # draw the rectangle composed of the four points

                # obtain the diagonal points of the rectangle
                pt1_x, pt1_y = box[0, 0], box[0, 1] # 외접사각형의 왼쪽위 꼭짓점
                pt3_x, pt3_y = box[2, 0], box[2, 1] # 외접사각형의 오른쪽 아래 꼭짓점
                # the center point of the line
                # 사각형의 중심 x, y 좌표
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                # 중심에 빨간 점을 찍음 (이미지, 중심좌표, 원 반지름, 원 색깔, 선두께)
                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # draw the center point
                
                #center_x 리스트에 저장
                center_x.append(line_center_x)
            else: # 컨투어가 없다면
                center_x.append(-1) # -1 저장
        for i in range(len(center_x)): # ROI 별 중심들 순회
            if center_x[i] != -1: # 순회 중 값이 -1이 아니라면
                if center_x[i] > max_center_x: # 값이 max_center보다 크다면
                    max_center_x = center_x[i] # max_center 값 갱신 --> 순회 후 최대 center_x 값 갱신
                centroid_sum += center_x[i] * self.rois[i][-1] # 중심좌표 * 해당 ROI 가중치를 더한다.
        if centroid_sum == 0: # 중심 좌표가 없다면
            return result_image, None, max_center_x # 처리된 이미지, None(각도 없음), 검출된 ROI 중 가장 오른쪽 중심점 x좌표
        
        # 가중평균으로 얻은 최종 중심 위치 = 중심좌표 모든 합 / 모든 ROI 가중치 합
        center_pos = centroid_sum / self.weight_sum  # calculate the center point based on the proportion

        # (w / 2.0) : 이미지 가로 중앙 좌표
        # (center_pos - (w / 2.0)) : 최종 중심점이 화면 중앙에서 얼마나 왼쪽/오른쪽으로 치우쳤는지
        # (h / 2.0) : 화면 세로 중심값으로 “기준 거리” 역할
        # atan : 기울기를 각도로 변환
        # angle : 라디안 -> 디그리로 변환, 부호 반전
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0)))
        
        return result_image, angle, max_center_x # 처리된 이미지, 각도, 가장 오른쪽

image_queue = queue.Queue(2) # 크기제한 2인 큐 생성

# 카메라 토픽 구독 시 콜백 함수
def image_callback(ros_image):
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8") # ROS 이미지 -> OpenCV 이미지 변환
    bgr_image = np.array(cv_image, dtype=np.uint8) # Numpay 배열로 어레이 (uint8 3채널)
    if image_queue.full(): # 이미지 큐가 꽉 찬다면
        # if the queue is full, remove the oldest image
        image_queue.get() # 오래된 이미지를 꺼내서 버림
        # put the image into the queue
    image_queue.put(bgr_image) # 그게 아니면 새 이미지를 큐에 넣음

# 차선 검출 수행 메인함수
def main():
    running = True # while 루프 제어 플래그
    # self.get_logger().info('\033[1;32m%s\033[0m' % (*tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max'])))

    while running: # 루프 시작
        try:
            image = image_queue.get(block=True, timeout=1) # 최신 이미지 가져오기 (1초 동안 대기)
        except queue.Empty: # 이미지가 안들어온다면 queue.Empty 발생
            if not running: # 큐가 비었으면 루프를 돌고, 종료 플래그가 들어오면 루프 해제
                break
            else:
                continue
        binary_image = lane_detect.get_binary(image) # 원본 이미지 -> 이진화 진행
        cv2.imshow('binary', binary_image) # 이진화 결과 볼 수 있게 창에 띄우기
        img = image.copy() # 원본 이미지 복사
        y = lane_detect.add_horizontal_line(binary_image) # 이진화 이미지에 차선 검출 수평선 추가
        roi = [(0, y), (640, y), (640, 0), (0, 0)] # ROI 영역 지정

        # ROI 영역을 검은색으로 채움 *** 아랫부분에 선이 남는지 확인
        cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])  # fill the top with black to avoid interference
        
        # 필터링 된 이진이미지 최대값(255) 좌표 x값 구하기
        min_x = cv2.minMaxLoc(binary_image)[-1][0]

        # min_x부터 오른쪽 끝까지 굵은 흰색 선 그리기
        cv2.line(img, (min_x, y), (640, y), (255, 255, 255), 50)  # draw a virtual line to guide the turning
        
        # 차선 검출 이미지, 조향각, ROI 중 최대 x 값 (얼마나 오른쪽에 치우쳤는지)
        result_image, angle, x = lane_detect(binary_image, image.copy()) 
        '''
        up, down = lane_detect.add_vertical_line_far(binary_image)
        #up, down, center = lane_detect.add_vertical_line_near(binary_image)
        cv2.line(img, up, down, (255, 255, 255), 10)
        '''
        cv2.imshow('image', img) # 원본+가이드라인 이미지 띄우기
        key = cv2.waitKey(1) # 종료 키 지정
        if key == ord('q') or key == 27:  # press Q or Esc to quit
            break

    cv2.destroyAllWindows() # 루프 종료시 윈도우 종료
    rclpy.shutdown() # ROS 종료

if __name__ == '__main__': # 이 스크립트를 직접 실행 했을 때
    import rclpy # ROS2 실행
    from sensor_msgs.msg import Image # Image 토픽 사용 준비
    rclpy.init() # ROS2 연결 준비 (초기화)
    node = rclpy.create_node('lane_detect') # lane_detect 노드 객체 생성
    lane_detect = LaneDetector('yellow') # 노란색 차선 인식 클래스 인스턴스 설정

    # 구독 추가 / 메세지 타입 : Image / 토픽이름 : /ascamera/camera_publisher/rgb0/image / 콜백 함수 / 큐사이즈
    node.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', image_callback, 1)

    # main 함수를 백그라운드에서 실행 (멀티 쓰레드)
    threading.Thread(target=main, daemon=True).start()

    # ROS 이벤트 루프 실행
    rclpy.spin(node)