#!/usr/bin/env python3 # 이 스크립트를 파이썬3 인터프리터로 실행하도록 하는 shebang
# encoding: utf-8 # 파일 인코딩 정보를 명시 (한글 주석 등 유니코드 문자 포함을 명확히 함)
# @data:2023/03/11 # 작성 또는 수정 날짜 메모
# @author:aiden # 작성자
# lane detection for autonomous driving # 자율주행을 위한 차선 감지(lane detection) 스크립트
import os # OS 환경 변수, 운영체제 관련 기능 사용
import cv2 # OpenCV 라이브러리 (영상 처리, 그리기, 컨투어 등)
import math # 수학 함수(atan, fabs, degrees 등) 사용
import queue # 스레드 간 이미지 전달용 Queue 사용
import threading # 영상 표시 루프와 ROS 스핀을 분리하기 위한 스레딩 사용
import numpy as np # 숫자 연산과 배열 처리
import sdk.common as common # 사용자/프로젝트 공용 유틸(sdk)에서 공통 함수 불러오기 (예: YAML 로드)
from cv_bridge import CvBridge # ROS 이미지(sensor_msgs/Image) ↔ OpenCV 이미지 변환 도구

bridge = CvBridge() # CvBridge 인스턴스 생성 (ROS 이미지 콜백에서 사용)

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
# LAB 색 범위를 담은 YAML 파일을 로드해 딕셔너리 형태로 획득
#lab_confg.yaml의 내용은 아래와 같다.
'''
lab:
  Mono:
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
# 차선(노란색) 검출 클래스
class LaneDetector(object):
    def __init__(self, color): # LaneDetecor 생성자, 내부 상태 초기화
        # lane color
        self.target_color = color # 검출 차선의 색상 이름 저장
        # ROI for lane detection
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera': # 불러온 카메라가 뎁스카메라라면
            # 관심영역(ROI) 지정(y1,y2,x1,x2,weight(중요도))
            # 중요도가 더 클수록 신뢰하도록

            self.rois = ((338, 360, 0, 320, 0.7), (292, 315, 0, 320, 0.2), (248, 270, 0, 320, 0.1))
            #카톡이미지참고
        else: # 그 카메라가 아니라면 다른 관심영역 지정
            self.rois = ((450, 480, 0, 320, 0.7), (390, 480, 0, 320, 0.2), (330, 480, 0, 320, 0.1))
        self.weight_sum = 1.0 # 가중합 계산 시 분모로 쓰일 값 초기화

    def set_roi(self, roi): # ROI 구성을 외부에서 특정값으로 바꾸고 싶을 때
        self.rois = roi

    @staticmethod
    # 가장 큰 컨투어 반환 함수 (최소면적 100픽셀 이상)
    def get_area_max_contour(contours, threshold=100):
        '''
        obtain the contour corresponding to the maximum area
        :param contours:
        :param threshold:
        :return:
        '''
        # 컨투어 목록에서 면적을 계산하고, threshold(최소 면적)보다 큰 것만 남긴 뒤 가장 큰 하나를 반환
        # cv2.countourArea() : 면적 계산 (픽셀 단위 크기)
        # math.fabs() : 음수를 절대값 처리
        # map(lamda c:... , countours): countours 리스트 요소들의 면적 계산
        # zip(): 원래 컨투어와 면적을 짝지어 튜플로 묶음
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        # 면적이 threshold를 넘는 컨투어만 필터링
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            # 남은 컨투어 중 면적이 최대인 항목 선택
            return max_c_a
        return None
            # 유효한 컨투어가 없으면 None 반환

    # 이진 이미지에서 오른쪽 절반을 확인해, 수평 가이드 라인(y 좌표)을 얻는 함수
    def add_horizontal_line(self, image):
        #   |____  --->   |————   ---> ——
        h, w = image.shape[:2] # 이미지 높이, 너비 가져오기
        roi_w_min = int(w/2) # 이미지 너비의 절반만 쓴다. (오른쪽)
        roi_w_max = w
        roi_h_min = 0
        roi_h_max = h
        # ROI로 잘라낸다. (오른쪽만 사용) (흰색과 검정만 있는 이미지)
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]  # crop the right half
        # ROI 위아래를 반전한다.
        flip_binary = cv2.flip(roi, 0)  # flip upside down
        # 반전된 ROI에서 최대값(255)의 위치 중 y좌표 획득 (가장 아래쪽 밝은 점의 y를 의미)
        # 카톡 사진 참고
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]  # extract the coordinates of the top-left point with a value of 255
        # 원래 좌표계 기준으로 변환된 y 위치 반환(수평 가이드 라인 y)
        return h - max_y

    # 원근상 ‘먼 곳’(상부/중앙 근처)의 수직 가이드 라인 두 점을 추정하는 함수
    def add_vertical_line_far(self, image):
        h, w = image.shape[:2]
        # 이미지 좌측 중상단 영역을 ROI로 설정
        roi_w_min = int(w/8)
        roi_w_max = int(w/2)
        roi_h_min = 0
        roi_h_max = h
        # ROI로 만든다.
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        # 상하좌우 반전(-1)으로 특정 기준에서 최대값 위치 탐색 용이화
        flip_binary = cv2.flip(roi, -1)  # flip the image horizontally and vertically
        # 디버그
        #cv2.imshow('1', flip_binary)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ret)
        # minVal：the minimum value
        # maxVal：the maximum value
        # minLoc：the location of the minimum value
        # maxLoc：the location of the maximum value
        # the order of traversal is: first rows, then columns, with rows from left to right and columns from top to bottom
        
        # flip된 ROI에서 최대값(보통 255)의 좌표 획득(먼 영역의 밝은 점 위치)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # extract the coordinates of the top-left point with a value of 255
        # 첫 최대점 아래로 55픽셀 떨어진 수평선 기준 재탐색 시작점 설정(경험적 오프셋)
        y_center = y_0 + 55
        # 해당 영역을 다시 ROI로 사용
        roi = flip_binary[y_center:, :]
        # 하위 ROI에서 다시 밝은 점 위치 탐색
        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        # 원본 좌표계로 되돌린 ‘아래쪽’ 기준점(down point) 계산
        down_p = (roi_w_max - x_1, roi_h_max - (y_1 + y_center))
        
        y_center = y_0 + 65
        roi = flip_binary[y_center:, :]
        (x_2, y_2) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x_2, roi_h_max - (y_2 + y_center))

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = (int((h - down_p[1])/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), h)

        return up_point, down_point

    def add_vertical_line_near(self, image):
        # ——|         |——        |
        #   |   --->  |     --->
        h, w = image.shape[:2]
        roi_w_min = 0
        roi_w_max = int(w/2)
        roi_h_min = int(h/2)
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # flip the image horizontally and vertically
        #cv2.imshow('1', flip_binary)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # extract the coordinates of the top-left point with a value of 255
        down_p = (roi_w_max - x_0, roi_h_max - y_0)

        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        y_center = int((roi_h_max - roi_h_min - y_1 + y_0)/2)
        roi = flip_binary[y_center:, :] 
        (x, y) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x, roi_h_max - (y + y_center))

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = down_p

        return up_point, down_point, y_center

    def get_binary(self, image):
        # recognize color through LAB space
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # convert RGB to LAB
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # Gaussian blur denoising
        mask = cv2.inRange(img_blur, tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # erode
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # dilate

        return dilated

    def __call__(self, image, result_image):
        # extract the center point based on the proportion
        centroid_sum = 0
        h, w = image.shape[:2]
        max_center_x = -1
        center_x = []
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # crop ROI
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # find contours
            max_contour_area = self.get_area_max_contour(contours, 30)  # obtain the contour with the largest area
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # the minimum bounding rectangle
                box = np.intp(cv2.boxPoints(rect))  # four corners
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (255, 255, 0), 2)  # draw the rectangle composed of the four points

                # obtain the diagonal points of the rectangle
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # the center point of the line
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # draw the center point
                center_x.append(line_center_x)
            else:
                center_x.append(-1)
        for i in range(len(center_x)):
            if center_x[i] != -1:
                if center_x[i] > max_center_x:
                    max_center_x = center_x[i]
                centroid_sum += center_x[i] * self.rois[i][-1]
        if centroid_sum == 0:
            return result_image, None, max_center_x
        center_pos = centroid_sum / self.weight_sum  # calculate the center point based on the proportion
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0))) # 화면 상에 얼마나 벗어났는지 보고 돌리는 값을 낸다
        
        return result_image, angle, max_center_x

image_queue = queue.Queue(2)
def image_callback(ros_image):
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    bgr_image = np.array(cv_image, dtype=np.uint8)
    if image_queue.full():
        # if the queue is full, remove the oldest image
        image_queue.get()
        # put the image into the queue
    image_queue.put(bgr_image)

def main():
    running = True
    # self.get_logger().info('\033[1;32m%s\033[0m' % (*tuple(lab_data['lab']['Stereo'][self.target_color]['min']), tuple(lab_data['lab']['Stereo'][self.target_color]['max'])))

    while running:
        try:
            image = image_queue.get(block=True, timeout=1)
        except queue.Empty:
            if not running:
                break
            else:
                continue
        binary_image = lane_detect.get_binary(image)
        cv2.imshow('binary', binary_image)
        img = image.copy()
        y = lane_detect.add_horizontal_line(binary_image)
        roi = [(0, y), (640, y), (640, 0), (0, 0)]
        cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])  # fill the top with black to avoid interference
        min_x = cv2.minMaxLoc(binary_image)[-1][0]
        cv2.line(img, (min_x, y), (640, y), (255, 255, 255), 50)  # draw a virtual line to guide the turning
        result_image, angle, x = lane_detect(binary_image, image.copy()) 
        '''
        up, down = lane_detect.add_vertical_line_far(binary_image)
        #up, down, center = lane_detect.add_vertical_line_near(binary_image)
        cv2.line(img, up, down, (255, 255, 255), 10)
        '''
        cv2.imshow('image', img)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # press Q or Esc to quit
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('lane_detect')
    lane_detect = LaneDetector('yellow')
    node.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', image_callback, 1)
    threading.Thread(target=main, daemon=True).start()
    rclpy.spin(node)