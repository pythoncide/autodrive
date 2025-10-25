# 🚗 ROS2 기반 자율주행 프로젝트

본 프로젝트는 **ROS2 Humble** 환경에서 **YOLOv5 객체 인식**과 **PID 기반 주행 제어**를 통합한 자율주행 로봇 시스템을 구현한 프로젝트 입니다.


## 🛠️ 기술 스택 (Tech Stack)
|구분|사용 기술|
|---|---|
|운영체제|`Ubuntu 22.04`|
|프레임워크|`ROS2 Humble`|
|언어|`Python 3.10`|
|비전 인식|`YOLOv5`, `OpenCV`|
|제어 로직|`PID Controller`, `MultiThreadedExecutor`|
|하드웨어|`RaspberryPi 4B`, `RRC Board`, `Breadboard (LED·LCD·Button)`|


## ⚙️ 주요 기능
- 차선 인식 주행: OpenCV 기반 ROI 분석 및 PID 제어
- 객체 인식 제어: 횡단보도, 신호등, 우회전, 주차 표지 등을 YOLOv5(ONNX)로 실시간 인식
- 주차 및 회전 로직: 인식 결과에 따른 상태 전환 및 각속도 제어
- 하드웨어 피드백: LED 색상 및 LCD 텍스트를 통한 주행 상태 표시

## ⚡ 주요 성과
- `.pt` → `.onnx` 변환을 통해 추론 속도 약 3배 향상
- ROS2 기반 병렬 처리로 인식–제어 간 실시간 동기화 구현
- PID 파라미터 튜닝을 통한 곡선 주행 안정성 확보

## 📌 주요 노드
영상 → 객체 인식 → FSM 판단 → 모터/LED/LCD 제어 로 이어지는 구조로 동작합니다.
- `/ascamera/camera_publisher` : 카메라 영상 publish
- `/yolov5_ros2` : ONNX 기반 객체 인식
- `/self_driving` : FSM + PID 제어
- `/ros_robot_controller` : 바퀴/모터 제어
- `/led_controller` : LED 하드웨어 제어
- `/lcd_controller` : LCD 출력

## 📂 참조 링크
- 🎥 시연 영상: [Youtube](https://youtu.be/eJriFnFVGAk)
- 📄 프로젝트 문서: [REPORT/](https://github.com/pythoncide/autodrive/tree/docs/report)

## 👥 기타 정보
- 팀원: [박현욱](https://github.com/Ian-Spangler), [성시경](https://github.com/GolbaengE), [염수림](https://github.com/rimit-rim)
- 프로젝트 기간: 25.09.24 ~ 25.10.13

---