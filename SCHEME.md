# ROS2 Robot Architecture

Загальна архітектура мобільного робота на базі ROS2 Jazzy (Raspberry Pi 5 + NixOS).

## 🏗️ Блок-схема

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  [КОНТРОЛЕР]                    [КАМЕРА]      [ЛІДА́Р LD06]     │
│  (клавіатура,keycontrol,        (lib/v4l2)    (USB 230400)      │
│   joystick)                                                      │
│      │                               │              │            │
│      │                               │              │            │
│      └───────────────────────────────┼──────────────┘            │
│                                      │                           │
│   /cmd_vel_raw (Twist)      /camera/image_raw (Image)           │
│         │                         │                              │
│         │                         │                              │
│         v                         v                              │
│   ╔════════════════════╗   ╔═══════════════════════╗             │
│   │ ld06_safety_node   │   │ camera_publisher_node │             │
│   │ (гальмування при   │   │ (публікує вихід)      │             │
│   │  перешкоді)        │   ╚═══════════════════════╝             │
│   ╚════════════════════╝            │                            │
│         │                           │                            │
│   /lidar/front_distance ←───┐      │                            │
│         │                    │      │                            │
│         v                    │      v                            │
│   /cmd_vel (Twist)          │   /camera/image_aruco             │
│    (відфільтровано)         │   (з маркерами)                   │
│         │                    │      │                            │
│         │                    │      │                            │
│         └────────────────────┘  [ВІДЕОПОТІК]                    │
│                │                                                 │
│                v                                                 │
│          ╔═════════════╗                                         │
│          │ motor_node  │                                         │
│          │ (мотори)    │                                         │
│          ╚═════════════╝                                         │
│                │                                                 │
│                v                                                 │
│            /odom (Odometry)                                      │
│         (позиція робота)                                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 📊 Топіки (Messages)

### Керування (Control)
```
/cmd_vel_raw
  ├─ Source: Контролер (keyboard, joystick)
  ├─ Type: geometry_msgs/Twist
  ├─ Fields: linear.x, angular.z
  └─ Subscriber: ld06_safety_node

/cmd_vel ⭐
  ├─ Source: ld06_safety_node (фільтровано за лідаром)
  ├─ Type: geometry_msgs/Twist
  ├─ Fields: linear.x, angular.z
  └─ Subscriber: motor_node
```

### Лідар (LiDAR)
```
/scan
  ├─ Source: ld06_node
  ├─ Type: sensor_msgs/LaserScan
  ├─ Fields: ranges[360], intensities[360]
  └─ Use: SLAM, navigation, mapping

/lidar/front_distance
  ├─ Source: ld06_node
  ├─ Type: std_msgs/Float32
  ├─ Value: дистанція спереду (м), або -1.0 якщо нема
  └─ Subscriber: ld06_safety_node (для гальмування)

/lidar/closest
  ├─ Source: ld06_node
  ├─ Type: std_msgs/Float32MultiArray
  ├─ Value: [distance_m, angle_deg]
  └─ Use: пошук найближчого об'єкту
```

### Камера (Camera)
```
/camera/image_raw
  ├─ Source: camera_publisher_node (libcamera/v4l2)
  ├─ Type: sensor_msgs/Image (BGR8)
  ├─ Resolution: 640x480 (за замовчуванням)
  └─ Subscriber: aruco_detector_node

/camera/image_aruco
  ├─ Source: aruco_detector_node
  ├─ Type: sensor_msgs/Image (BGR8)
  ├─ Content: відео з намальованими ArUco маркерами
  └─ Use: RViz2, rqt_image_view
```

### Одометрія (Odometry)
```
/odom
  ├─ Source: motor_node
  ├─ Type: nav_msgs/Odometry
  ├─ Fields:
  │  ├─ pose.position: (x, y, theta)
  │  └─ twist.linear.x, twist.angular.z
  └─ Use: SLAM, localization, navigation
```

### Safety
```
/safety/lidar_stop
  ├─ Source: ld06_safety_node
  ├─ Type: std_msgs/Bool
  ├─ Value: true = гальмування активне
  └─ Use: індикація, логування
```

## 🔗 Топіки за Пакетом

### ld06_lidar 📦
```
Слухає:     /cmd_vel_raw
Публікує:   /scan
            /lidar/front_distance
            /lidar/closest
            /safety/lidar_stop
```

### aruco_detector 📦
```
Слухає:     /camera/image_raw
Публікує:   /camera/image_aruco
            Логи детектованих маркерів
```

### diff_drive 📦
```
Слухає:     /cmd_vel
Публікує:   /odom
```

### Контролер (зовнішній) 📦
```
Публікує:   /cmd_vel_raw
```

## 🔄 Потік Даних

### 1️⃣ Керування Мотором
```
Kontroller
    ↓ /cmd_vel_raw
ld06_safety_node
    ↓ (гальмує при перешкоді)
/cmd_vel
    ↓
motor_node
    ↓ GPIO PWM
🛞 Мотори
```

### 2️⃣ LiDAR Safety
```
ld06_node
    ↓ /lidar/front_distance
ld06_safety_node
    (гальмує якщо distance ≤ stop_distance_m)
```

### 3️⃣ Локалізація (Odometry)
```
motor_node
    ↓ /odom (x, y, theta)
SLAM алгоритм (якщо є)
    ↓
Карта + Позиція
```

### 4️⃣ Відео (Optional)
```
camera_publisher_node
    ↓ /camera/image_raw
aruco_detector_node
    ↓ /camera/image_aruco
RViz2 / rqt_image_view
```

## 🎯 Критичні Шляхи

### ✅ Необхідне для руху
```
1. Контролер → /cmd_vel_raw ✓
2. ld06_safety_node слухає /cmd_vel_raw ✓
3. motor_node слухає /cmd_vel ✓
4. GPIO мотори працюють ✓
```

### ⚠️ Failsafe Механізми
```
1. Таймаут команди (0.5 сек) → STOP
2. Таймаут лідара (0.7 сек) → STOP
3. Дистанція < 0.22м → гальмування
4. Гістерезис (0.22м → 0.30м) → запуск
```

## 📈 Частоти (Hz)

```
/cmd_vel_raw        контролер → залежно від типу
/cmd_vel            ld06_safety_node (100 Hz, timer 10ms)
/scan               ld06_node (50 Hz за замовчуванням)
/lidar/front_distance  ld06_node (50 Hz)
/camera/image_raw   camera_publisher_node (30 Hz)
/camera/image_aruco aruco_detector_node (30 Hz)
/odom               motor_node (100 Hz)
```

## 🚀 Запуск (Full System)

```bash
# Все в одному
ros2 launch ld06_lidar ld06_launch.py &
ros2 run aruco_detector camera_publisher_node &
ros2 run aruco_detector aruco_detector_node &
ros2 run diff_drive motor_node &
ros2 run your_teleop your_controller

# Або по один раз
ros2 launch ld06_lidar ld06_launch.py port:=/dev/ttyUSB0
ros2 launch aruco_detector aruco_launch.py
ros2 run diff_drive motor_node
```

## 📋 Залежності Топіків

```
/cmd_vel_raw
    ├─ Обов'язкова для: ld06_safety_node
    ├─ Джерело: контролер
    └─ Без неї: robot не отримує команди

/lidar/front_distance
    ├─ Обов'язкова для: ld06_safety_node (гальмування)
    ├─ Джерело: ld06_node
    └─ Без неї: safety відключається

/camera/image_raw
    ├─ Опціональна для: aruco_detector_node
    ├─ Джерело: camera_publisher_node
    └─ Без неї: немає детекції маркерів

/cmd_vel
    ├─ Обов'язкова для: motor_node
    ├─ Джерело: ld06_safety_node
    └─ Без неї: мотори не рухаються
```

## 🔌 Фізичні Підключення

```
┌─ Raspberry Pi 5
│
├─ USB 0: LD06 LiDAR (230400 baud)
├─ GPIO: мотори (ENA/IN1/IN2, ENB/IN3/IN4)
├─ USB 1: Web Camera (libcamera або /dev/video0)
├─ Мережа: SSH, ROS2 messages
│
└─ Живлення: 5V до RPi, 12V до моторів (через L298N)
```

## 📚 Залежності Пакетів

```
ld06_lidar
  ├─ Залежить від: libserialport
  └─ Публікує для: diff_drive, aruco_detector

aruco_detector
  ├─ Залежить від: OpenCV, GStreamer
  └─ Публікує для: (RViz2, rqt)

diff_drive
  ├─ Залежить від: lgpio
  └─ Публікує для: SLAM алгоритми

Контролер
  ├─ Залежить від: joy, teleop_twist_keyboard
  └─ Публікує для: ld06_safety_node
```

## 🎮 Типові Сценарії

### Сценарій 1: Автономна навігація з LiDAR
```
1. Контролер → /cmd_vel_raw
2. ld06_safety_node → гальмує при перешкоді
3. motor_node → рухається + одометрія
4. SLAM (якщо є) → будує карту на основі /scan
```

### Сценарій 2: Слідування маркерам
```
1. camera_publisher_node → /camera/image_raw
2. aruco_detector_node → знаходить маркер
3. Контролер (custom) → /cmd_vel_raw (на основі маркера)
4. ld06_safety_node → гальмує + safety
```

### Сценарій 3: Телеоперація з гальмуванням
```
1. Joystick → /cmd_vel_raw
2. ld06_safety_node → гальмує близьке
3. motor_node → рухається
4. /odom → позиція робота
```
