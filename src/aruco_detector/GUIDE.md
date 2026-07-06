# ArUco Detector - ROS2 Camera & Marker Detection

Камера + ArUco маркер детектор для ROS2. Публікує відео з детектованими маркерами.

## Запуск

```bash
# Обидва ноди (камера + детектор)
ros2 launch aruco_detector aruco_launch.py

# Окремо
ros2 run aruco_detector camera_publisher_node
ros2 run aruco_detector aruco_detector_node
```

## Архітектура

```
[Camera] (libcamera або v4l2)
    ↓
[camera_publisher_node]
    ↓
/camera/image_raw (RGB вихід)
    ↓
[aruco_detector_node]
    ↓
/camera/image_aruco (з маркерами)
```

## Ноди

### camera_publisher_node
Публікує вихід камери в ROS2.

**Топіки:**
- `/camera/image_raw` — sensor_msgs/Image (BGR8)

**Параметри:**
```yaml
camera_source: "libcamera"    # "libcamera" (Pi 5) або "v4l2"
width: 640                    # Ширина в пікселях
height: 480                   # Висота в пікселях
fps: 30                       # Кадрів на секунду
```

**Запуск з параметрами:**
```bash
ros2 run aruco_detector camera_publisher_node \
  --ros-args \
  -p camera_source:=libcamera \
  -p width:=1280 \
  -p height:=720 \
  -p fps:=30
```

### aruco_detector_node
Детектує ArUco маркери в потоці камери.

**Топіки:**
- **Слухає:** `/camera/image_raw` (від camera_publisher)
- **Публікує:** `/camera/image_aruco` — Image з намальованими маркерами

**Вихід у логи:**
```
[aruco_detector_node] [INFO] Detected 2 marker(s)
[aruco_detector_node] [INFO] Marker ID: 0
[aruco_detector_node] [INFO] Marker ID: 5
```

## Дебаг

```bash
# Слухай вихід камери
ros2 topic echo /camera/image_raw

# Слухай вихід детектора
ros2 topic echo /camera/image_aruco

# Частота кадрів
ros2 topic hz /camera/image_raw      # Мав б бути ~30 Гц
ros2 topic hz /camera/image_aruco    # Мав б бути ~30 Гц

# Параметри camera_publisher
ros2 param list /camera_publisher_node
ros2 param get /camera_publisher_node camera_source
```

## Тестування

```bash
# 1. Запусти ноди
ros2 launch aruco_detector aruco_launch.py

# 2. Перегляди вихід (потребує RViz2 або rqt_image_view)
ros2 run rqt_image_view rqt_image_view

# 3. Вибери /camera/image_aruco → бачиш маркери в реальному часі

# 4. Або просто слухай логи
ros2 run aruco_detector aruco_detector_node --ros-args --log-level DEBUG
```

## Залежності (Nix)

```nix
buildInputs = [
  pkgs.opencv,      # OpenCV з ArUco
  pkgs.gstreamer,   # GStreamer для camera
];
```

## Типові проблеми

### Камера не відкривається
```bash
# Перевір чи камера підключена
ls -la /dev/video*

# Логи з деталями
ros2 run aruco_detector camera_publisher_node --ros-args --log-level DEBUG
```

### Нема маркерів у виході
- Переконайся що маркер в кадрі
- Маркер має бути чітким і контрастним
- Спробуй підірватися AprilTags замість ArUco

### Низька частота кадрів
- Зменш resolution: `width:=320 height:=240`
- Збільш fps: `fps:=15` (якщо камера це підтримує)

## Файли

```
aruco_detector/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── aruco_launch.py
├── src/
│   ├── camera_publisher_node.cpp
│   └── aruco_detector_node.cpp
```

## Залежності ROS2

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>opencv</depend>
```
