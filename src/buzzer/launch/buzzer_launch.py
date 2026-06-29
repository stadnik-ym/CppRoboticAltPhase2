from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="buzzer_node",
            executable="buzzer_node",
            name="buzzer_node",
            output="screen",
            parameters=[{
                # gpiochip0 on Raspberry Pi 5 — check with `gpiodetect`
                "gpio_chip": "gpiochip0",
                # BCM pin number (e.g. 17 = physical pin 11)
                "gpio_line": 17,
                # How long to beep when True is received (milliseconds)
                "beep_duration_ms": 500,
                # Software PWM frequency for passive buzzers (Hz).
                # For active buzzers set this very high (e.g. 10000) —
                # they only need a logic HIGH, not a square wave.
                "beep_frequency_hz": 2000,
            }],
            remappings=[
                # ("buzzer", "/your/custom/topic"),
            ],
        )
    ])
