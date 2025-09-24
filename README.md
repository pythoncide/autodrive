# autodrive

### 📌 브랜치 전략

- `main`: 운영 배포용
- `feature/{기능이름}`: 기능 추가
- `fix/{버그설명}`: 버그 수정
- `refactor/{리팩토링내용}`: 리팩토링 작업

### 🔖 커밋 태그

| 태그 | 설명 |
|------|------|
| ✨ Feat | 기능 추가 |
| 🐛 Fix | 버그 수정 |
| ♻️ Refactor | 리팩토링 |
| 📄 Docs | 문서 작업 |
| 🔧 Chore | 설정 변경 |
| ✅ Test | 테스트 코드 |
| 🗃️ Rename | 파일/폴더명 변경 |
| 🚀 Deploy | 배포 관련 |



# YOLOv5 ROS2 Subscription Node

## Set up
```
python3 -m venv .venv
source .venv/bin/activate
(.venv) pip install -r requirements.txt
```

## Build ROS2 workspace
```
(.venv) cd ros2_ws
(.venv) colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(which python3)" -DPYTHON_EXECUTABLE="$(which python3)"
```

## Run YOLOv5_ROS2
* Terminal 1
```
(.venv) source ./install/setup.bash
(.venv) ros2 run image_tools cam2image
```

* Terminal 2
```
(.venv) source ./install/setup.bash
(.venv) ros2 run yolov5_ros2 yolo_detect \
  --ros-args \
  -p device:=cpu \
  -p pub_result_img:=true \
  -p image_topic:=/image
```

* Terminal 3
```
(.venv) source ./install/setup.bash
(.venv) ros2 run yolosub subscribe
```