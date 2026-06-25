# Stage 1: Builder
FROM ros:jazzy as builder

WORKDIR /ws

# Установка зависимостей для сборки
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libopencv-dev \
    libserial-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Копируем исходный код
COPY src ./src

# Инициализируем зависимости ROS
RUN rosdep init && rosdep update

# Устанавливаем зависимости пакетов
RUN rosdep install --from-paths src --ignore-src -y

# Сборка проекта
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Stage 2: Runtime
FROM ros:jazzy-runtime

WORKDIR /ws

# Минимальные runtime зависимости
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-core4.8d \
    libopencv-imgproc4.8d \
    libserial2 \
    libpcl-core1.13 \
    python3-minimal \
    && rm -rf /var/lib/apt/lists/*

# Копируем только install директорию из builder-образа
COPY --from=builder /ws/install ./install

# Setup скрипт
RUN echo "#!/bin/bash\nset -e\n. /opt/ros/jazzy/setup.sh\n. ./install/setup.sh\nexec \"\$@\"" > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
