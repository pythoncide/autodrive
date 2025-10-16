# ROS2 기반 자율주행 로봇카 프로젝트
본 프로젝트는 **ROS2 Humble** 환경에서 **YOLOv5 객체 인식**과 **PID 기반 주행 제어**를 통합한 자율주행 로봇카 시스템을 구현한 프로젝트 입니다.

## 🛠️ 기술 스택 (Tech Stack)
|구분|사용 기술|
|---|---|
|운영체제|Ubuntu 22.04|
|프레임워크|ROS2 Humble|
|언어|Python 3.10|
|비전 인식|YOLOv5 (ONNX), OpenCV|
|제어 로직|PID Controller, MultiThreadedExecutor|
|하드웨어|Raspberry Pi 4B, RRC Board, Breadboard (LED·LCD·Button)|

## 🚀 프로젝트 요약
- YOLOv5s(.pt) 모델의 추론 지연 문제를 해결하기 위해 ONNX 모델로 변환하였습니다.
- ROS2 멀티스레드 구조를 적용하여 객체 인식과 주행 제어를 병렬 처리하였습니다.
- PID 제어를 통해 차선 중심을 유지하며 곡선 구간에서도 안정적인 주행이 가능하도록 하였습니다.
- LED·LCD·버튼을 활용한 실시간 주행 상태 피드백 기능을 구현하였습니다.

## 📂 참고 링크
- 🎥 시연 영상: [Youtube](https://youtu.be/eJriFnFVGAk)
- 