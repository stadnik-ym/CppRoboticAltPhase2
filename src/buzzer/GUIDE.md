# buzzer_node

ROS 2 Jazzy нода для керування пищалкою через GPIO (libgpiod v2 C++ API).

## Топіки

| Топік | Тип | Опис |
|-------|-----|------|
| `buzzer` (sub) | `std_msgs/Bool` | `true` — пискнути, `false` — зупинити |

## Параметри

| Параметр | Тип | За замовчуванням | Опис |
|----------|-----|-----------------|------|
| `gpio_chip` | string | `gpiochip4` | GPIO-чіп (`gpiodetect` для перевірки; на RPi 5 зазвичай `gpiochip4`) |
| `gpio_line` | int | `18` | BCM-номер лінії |
| `beep_duration_ms` | int | `500` | Тривалість одного писку, мс |
| `beep_frequency_hz` | int | `2000` | Частота software PWM — для пасивних пищалок; для активних ставити `10000`+ |

## Залежності

```
libgpiod >= 2.0   (пакет: libgpiod-dev; Nix: pkgs.gpiod)
```

## Запуск

```bash
ros2 launch buzzer buzzer_launch.py

# або напряму
ros2 run buzzer buzzer_node \
  --ros-args -p gpio_chip:=gpiochip4 -p gpio_line:=18
```

## Тест

```bash
ros2 topic pub --once /buzzer std_msgs/msg/Bool "data: true"
```

## Інтеграція з lidar_safety

`LidarSafetyNode` публікує в `/buzzer` з різними інтервалами залежно від стану:

| Стан | Інтервал |
|------|----------|
| Перешкода попереду (заблоковано) | 2 с |
| Timeout cmd або lidar | 0.5 с |
| Сповільнення (slowdown) | 4 с |

## Нотатки

- **Активна пищалка** (вбудований генератор) — потребує лише HIGH сигнал, `beep_frequency_hz` не має значення.
- **Пасивна пищалка** — потребує PWM, налаштовується через `beep_frequency_hz`.
- Права на GPIO: додати користувача до групи `gpio` (`sudo usermod -aG gpio $USER`).
