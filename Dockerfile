#!/bin/bash
set -e

# Сорсим глобальный ROS2 внутри контейнера
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Сорсим наш собранный воркспейс, если он существует
if [ -f "/node_ws/install/setup.bash" ]; then
  source "/node_ws/install/setup.bash"
fi

exec "$@"
