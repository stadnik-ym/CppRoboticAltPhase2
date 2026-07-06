# LD06 Lidar (C++)

Драйвер LD06 LiDAR для ROS2. Читає 360° сканування, публікує LaserScan і safety layer для гальмування при перешкодах.

## Запуск

```bash
# Лідар + safety (обидві ноди)
ros2 launch ld06_lidar ld06_launch.py port:=/dev/ttyUSB0

# З кастомним baudrate
ros2 launch ld06_lidar ld06_launch.py port:=/dev/ttyUSB0 baudrate:=230400

# Окремі ноди
ros2 run ld06_lidar ld06_node --ros-args -p port:=/dev/ttyUSB0
ros2 run ld06_lidar ld06_safety_node
```

## Архітектура

```
/dev/ttyUSB0
    ↓
[ld06_node] — читає пакети LD06, парсить, публікує
    ↓
    ├→ /scan (sensor_msgs/LaserScan)
    ├→ /lidar/front_distance (Float32)
    └→ /lidar/closest (Float32MultiArray)
        ↓
    [ld06_safety_node] — фільтрує команди з лідара
        ↓
        ├← /cmd_vel_raw (від контролера)
        └→ /cmd_vel (до motor_node)
```

## Топіки

### LD06 Node публікує:
- **`/scan`** — `sensor_msgs/LaserScan`
  - 360° лазерне сканування
  - Публікує 50 раз/сек за замовчуванням

- **`/lidar/front_distance`** — `std_msgs/Float32`
  - Мінімальна дистанція в передній зоні (градуси -35…+35)
  - -1.0 = немає валідної точки

- **`/lidar/closest`** — `std_msgs/Float32MultiArray`
  - [distance_m, angle_deg] — найближча точка на 360°

### Safety Node трансформує:
- **Слухає:** `/cmd_vel_raw` (команди з keycontrol/joystick)
- **Слухає:** `/lidar/front_distance` (від ld06_node)
- **Публікує:** `/cmd_vel` → motor_node
  - Гальмує лінійну швидкість при перешкоді
  - Дозволяє ротацію / реверс за параметром

## Параметри Launch

### LD06 Node
```yaml
port: /dev/ttyUSB0              # Serial port (за замовчуванням USB0)
baudrate: 230400                # Baud rate LD06
frame_id: laser_frame           # TF frame ID
publish_rate_hz: 50.0           # Частота публікування
angle_resolution_deg: 1.0       # 1°/точка (360 бінів)
angle_offset_deg: 0.0           # Калібрування кута
invert_angle_direction: false   # Дзеркально розвернути?
front_min_deg: -35.0            # Передня зона (ліво)
front_max_deg: 35.0             # Передня зона (право)
point_max_age_sec: 0.35         # Як довго точка залишається валідною
front_min_points: 1             # Мін. точок у передній зоні
```

### Safety Node
```yaml
input_cmd_topic: /cmd_vel_raw           # Звідки читати команди
output_cmd_topic: /cmd_vel              # Куди писати команди
front_distance_topic: /lidar/front_distance

cmd_timeout_sec: 0.5                    # Таймаут команди → STOP
lidar_timeout_sec: 0.7                  # Таймаут лідара → STOP

stop_distance_m: 0.22                   # На якій дистанції зупинити
clear_distance_m: 0.30                  # На якій очистити (гістерезис)

allow_rotation_when_blocked: true       # Дозволити крутитися на місці
allow_reverse_when_blocked: true        # Дозволити реверс

max_linear_x: 0.35                      # Макс. лінійна швидкість (м/с)
max_angular_z: 1.5                      # Макс. кутова швидкість (рад/с)

enable_slowdown: true                   # Плавне сповільнення перед перешкодою
slowdown_distance_m: 0.80               # З якої дистанції почати сповільнювати
min_slowdown_factor: 0.25               # Мінімальна швидкість (25%)
```

## Дебаг

```bash
# Список всіх топіків
ros2 topic list

# Слухай дистанцію спереду
ros2 topic echo /lidar/front_distance

# Слухай весь скан
ros2 topic echo /scan

# Частота публікування
ros2 topic hz /scan                     # Мав б бути ~50 Гц

# Параметри вузла
ros2 param list /ld06_node
ros2 param get /ld06_node angle_offset_deg
ros2 param set /ld06_node angle_offset_deg 5.0    # Динамічна зміна

# Логи з деталями
ros2 run ld06_lidar ld06_node --ros-args --log-level ld06_node:=DEBUG
ros2 run ld06_lidar ld06_safety_node --ros-args --log-level lidar_safety:=DEBUG
```

## Тестування

```bash
# 1. Запусти лідар
ros2 launch ld06_lidar ld06_launch.py port:=/dev/ttyUSB0

# 2. В іншому терміналі слухай команди
ros2 topic echo /cmd_vel

# 3. В третьому публікуй команду вручну
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Якщо спереду нема перешкоди → /cmd_vel матиме 0.1 м/с
# Якщо перешкода ≤ 0.22м → /cmd_vel буде 0.0 (гальмування)
```

## Калібрування

### Кут неправильний
```bash
# Робот знає що спереду, але лідар каже що ліворуч?
ros2 param set /ld06_node angle_offset_deg 45.0
```

### Ліво/право міняються
```bash
ros2 param set /ld06_node invert_angle_direction true
```

### Передня зона не та
```bash
# Розширити зону (наприклад 50 градусів)
ros2 param set /ld06_node front_min_deg -50.0
ros2 param set /ld06_node front_max_deg 50.0
```

## Проблеми

### Нема даних на `/scan`
```bash
# Перевір USB порт
ls -la /dev/ttyUSB*

# Перевір що лідар підключений
ros2 topic list | grep lidar

# Логи
ros2 run ld06_lidar ld06_node --ros-args --log-level DEBUG
```

### Safety зупиняє робота без причини
- Перевір що контролер публікує на `/cmd_vel_raw`
- Перевір таймаути (за замовчуванням 0.5 сек для команд)

### Робот не гальмує при перешкоді
```bash
# Перевір що safety слухає правильний топік
ros2 param get /lidar_safety front_distance_topic

# Перевір дистанції гальмування
ros2 param get /lidar_safety stop_distance_m
ros2 param get /lidar_safety clear_distance_m
```

## Файли в пакеті

```
ld06_lidar/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── ld06_launch.py              # Запуск обох нодів
├── include/ld06_lidar/
│   ├── ld06_ros_node.hpp
│   └── ld06_safety_node.hpp        
└── src/
    ├── ld06_ros_node.cpp
    ├── ld06_safety_node.cpp
```
