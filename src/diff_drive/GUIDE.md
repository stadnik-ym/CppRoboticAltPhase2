# Diff Drive - ROS2

Керування диференціальним приводом (2 мотори) для мобільного робота. PWM керування з компенсацією перекосу моторів.

## Запуск

```bash
# Запусти ноду
ros2 run diff_drive motor_node

# Або через launch (якщо є)
ros2 launch diff_drive diff_drive_launch.py
```

## Архітектура

```
/cmd_vel (Twist)
    ↓
[motor_node]
    ├→ GPIO PWM (ліво/право мотори)
    └→ /odom (Odometry)
```

## Топіки

### Слухає:
- **`/cmd_vel`** — `geometry_msgs/msg/Twist`
  - `linear.x` — лінійна швидкість (м/с, -0.2…+0.2)
  - `angular.z` — кутова швидкість (рад/с, -2.0…+2.0)

### Публікує:
- **`/odom`** — `nav_msgs/msg/Odometry`
  - Положення робота (x, y, theta)
  - Швидкість руху та ротації

## Параметри (константи в коді)

### PWM & Мотори
```cpp
PWM_FREQ              700         // Частота PWM (Гц)
LEFT_MIN_PWM          90          // Мінімальний PWM для лівого мотора (%)
RIGHT_MIN_PWM         90          // Мінімальний PWM для правого мотора (%)
```

### Компенсація (для рівного руху вперед)
```cpp
LEFT_GAIN             1.00        // Загальна компенсація лівого мотора
RIGHT_GAIN            1.00        // Загальна компенсація правого мотора
LEFT_FORWARD_GAIN     1.00        // Коефіцієнт для руху вперед (ліво)
RIGHT_FORWARD_GAIN    1.00        // Коефіцієнт для руху вперед (право)
LEFT_BACKWARD_GAIN    1.00        // Коефіцієнт для руху назад (ліво)
RIGHT_BACKWARD_GAIN   1.00        // Коефіцієнт для руху назад (право)
LEFT_INVERT           false       // Розвернути лівий мотор?
RIGHT_INVERT          false       // Розвернути правий мотор?
```

### Робот
```cpp
WHEEL_BASE            0.16        // Відстань між колесами (м)
MAX_LINEAR            0.2         // Макс. лінійна швидкість (м/с)
MAX_ANGULAR           2.0         // Макс. кутова швидкість (рад/с)
CMD_TIMEOUT           0.5         // Таймаут команди → STOP (сек)
```

### GPIO Пини
```cpp
ENA   17              // PWM для лівого мотора
IN1   27              // Напрямок ліво (1)
IN2   22              // Напрямок ліво (2)
ENB   13              // PWM для правого мотора
IN3   26              // Напрямок право (1)
IN4   19              // Напрямок право (2)
```

## Дебаг

```bash
# Слухай команди
ros2 topic echo /cmd_vel

# Слухай одометрію
ros2 topic echo /odom

# Частота
ros2 topic hz /cmd_vel
ros2 topic hz /odom

# Логи
ros2 run diff_drive motor_node --ros-args --log-level DEBUG
```

## Тестування

```bash
# 1. Запусти мотор ноду
ros2 run diff_drive motor_node

# 2. В іншому терміналі публікуй команди
# Рух вперед на 0.1 м/с
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Ротація проти годинникової стрілки
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'

# Комбіновано (дуга)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

## Калібрування

### Робот їде вліво при рухе вперед
- Лівий мотор повільніший → `LEFT_FORWARD_GAIN` > `RIGHT_FORWARD_GAIN`
- Наприклад: `LEFT_FORWARD_GAIN = 1.1`, `RIGHT_FORWARD_GAIN = 1.0`

### Робот їде вправо при рухе вперед
- Правий мотор повільніший → `RIGHT_FORWARD_GAIN` > `LEFT_FORWARD_GAIN`

### Коліші ривкаються при старті
- Збільш MIN_PWM: `LEFT_MIN_PWM = 100`, `RIGHT_MIN_PWM = 100`

### Мотори не крутяться
- Перевір GPIO пини: `ls -la /dev/gpiochip*`
- Перевір che напруги живлення

## Одометрія

Ноде інтегрує швидкість для обчислення позиції:

```
x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += w * dt
```

Точність залежить від:
- Вимірювання швидкості (ми інтегруємо дані команд)
- Синхронізації моторів (потребує калібрування)
- Проковзування коліс

Для точної одометрії розглянь додавання енкодерів.

## Типові проблеми

### Мотори не крутяться
```bash
# Перевір GPIO
gpioinfo | grep -E "17|27|22|13|26|19"

# Перевір пини в коді (може відрізнятися на різних Raspberry Pi)
```

### Робот крутиться на місці замість руху вперед
- Обидва мотори крутяться в протилежні боки
- Встав `LEFT_INVERT = true` або `RIGHT_INVERT = true`

### Одна колеса швидша
- Калібруй GAIN коефіцієнти (див. вище)

### Рапучі/ривки при старті
- Збільш MIN_PWM (мінімальна напруга для руху)

## Файли

```
diff_drive/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── motor_node.cpp
```

## Залежності (Nix)

```nix
buildInputs = [
  pkgs.lgpio,      # GPIO контроль (замість RPi.GPIO)
];
```

## Залежності ROS2

```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```
