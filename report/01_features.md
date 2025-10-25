# 🚗 주요 기능 구현

본 프로젝트에서는 차선 인식 주행, 객체 인식 제어, 주차 및 회전 로직, 하드웨어 피드백 제어(LED·LCD·버튼) 의 네 가지 기능을 중심으로 **자율주행 로봇 시스템**을 구현했습니다.  
각 기능은 ROS2 노드 단위로 분리되어 병렬적으로 동작하며, SelfDriving 노드가 모든 정보를 통합하여 주행 명령을 결정합니다.

## 🛣️ 1. 차선 인식 주행 (Lane Detection)
카메라 입력을 기반으로 OpenCV 영상처리를 수행하여 차선을 검출하고,
PID 제어를 통해 차량이 차선을 따라 주행하도록 합니다.

### 처리 흐름
1. ROI 설정 후 Binary Mask 생성
2. Contour 기반 차선 중심 좌표 추출
3. 오프셋(offset) 계산
4. 오차에 PID 적용하여 조향각(angular.z) 결정
5. 선형 속도(linear.x)는 상황에 따라 감속/유지

### 특징
- ROI 기반 가중치 합산 방식으로 안정적 중심점 계산
- 커브 구간 진입 시 속도 자동 감속
- PID 수치 (코드 기준): `PID(0.28, 0.0, 0.04)`

## 🚦 2. 객체 인식 기반 제어 (Object Detection)
YOLOv5 모델을 활용하여 횡단보도, 신호등, 주차·우회전 표지판 등의 객체를 탐지하고,
인식 결과에 따라 주행 상태를 변경합니다.

### 작동 방식
- YOLOv5_ROS2 노드에서 카메라 영상을 입력받아 실시간 추론 수행
- 인식 결과를 `/yolov5_ros2/object_detect` 토픽으로 송신
- `SelfDriving` 노드가 이를 구독하여 상태 전환을 결정

### 인식 클래스 (`self_driving.py` 코드 기준)
| 클래스명 | 의미 | FSM 동작 유무 |
|---|---|---|
| `cross_walk` | 횡단보도 | ✅ (일시 정지) |
| `light_red` | 빨간불 | ✅ (정지 유지) |
| `light_green` | 초록불 | ✅ (출발/통과) |
| `light_yellow` | 노란불 | ✅ (상황 유지) |
| `parking` | 주차 표지 | ✅ (주차 FSM) |
| `right` | 우회전 표지 | ✅ (우회전 FSM) |
| `straight` | 직진 표지 | ❌ (행동 없음, 인식만 함) |

## 🅿️ 3. 주차 및 회전 로직 (Parking & Turning)
YOLO 인식 이벤트에 따라 상태 기반 FSM으로 행동을 수행합니다.

### 우회전 FSM
`idle → forward → turning → idle`

**조건:**
- `right` 일정 프레임 이상 연속 감지 시 forward 단계 진입  
- 이후 선회 시간만큼 조향 유지 후 PID 주행 복귀

### 주차 FSM
`idle → forward → strafe → done`

**조건:**
- `parking` 감지 시 주차 시퀀스 진입  
- 전진 → Y축 이동 → LED 점멸 → 주차 완료
- 주차 후 LED가 자동으로 빨강색으로 변경되어 상태 표시

## 🚏 4. 횡단보도 & 신호등 FSM

횡단보도와 신호등은 상호 간섭이 있기 때문에 FSM 기반으로 처리합니다.

### 횡단보도 FSM
`idle → stopping(정지) → cooldown(일정 시간 무시) → idle`

### 신호등 처리
- `light_red` → 정지 유지
- `light_green` → 출발
- `light_yellow` → 유지
- 횡단보도 FSM과 충돌 시, **횡단보도 이벤트 우선 처리**

## 💡 5. 하드웨어 피드백 제어 (LED·LCD·Button)
### LED 제어
- ROS2 토픽 `/led_cmd` 구독 (led_controller 노드)
- SelfDriving FSM 상태에 따라 Publish
- 점멸은 self_driving 타이머 기반, 제어는 led_controller에서 수행

| 상태   | 색상    | 의미       |
| ---- | ----- | -------- |
| 주행 중 | 🟢 초록 | 정상 주행    |
| 회전 중 | 🟡 노랑 | 우회전 동작 중 |
| 정지   | 🔴 빨강 | 대기/주차 상태 |

### LCD 출력
- ROS2 토픽 `/ui/lcd` 구독 (lcd_controller 노드)
- SelfDriving에서 문자열 Publish → LCD 표시

### 버튼 입력
- ROS2 메시지 `/button_state`를 통해 입력 감지
- 버튼 1회 클릭 시 → 주행 시작 / 다시 클릭 시 → 정지

> 이 하드웨어 제어는 모두 **브레드보드 + 라즈베리파이 GPIO** 기반으로 구현했습니다.

## ✨ 통합 구조
```
Camera
 ┣━ YOLOv5_ROS2 → /yolov5_ros2/object_detect
 ┣━ LaneDetect → lane_x, lane_angle
 ┗━ SelfDriving(FSM)
       ┣━ PID 제어
       ┣━ right / parking / cross_walk / light_* 처리
       ┗━ LED, LCD, Button 제어
```
본 기능들을 통해 차량은 **라인 추종–객체 인식–상태 이벤트 제어–피드백 출력**이 모두
동기적으로 동작하는 자율주행을 수행합니다.

---