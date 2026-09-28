# Образ для симуляции Robomarvel: ROS 2 Jazzy, терминал + Foxglove/Lichtblick.
# GUI внутри контейнера не используется.

# Тот же базовый образ, что на роботе (Dockerfile.rbm, там он запинен дайджестом
# sha256:c5705f61 = jazzy-ros-base). desktop-full не нужен: симуляция чисто питоновская,
# Gazebo/RViz не используются, а всё транзитивно нужное ставится явно ниже.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# apt-get upgrade обязателен и должен быть в одном слое с install: базовый образ -
# снимок на дату сборки, пакеты ниже приезжают свежими из packages.ros.org.
# Зависимости в ROS объявлены нестрого, поэтому ABI расходится молча и ноды падают
# с undefined symbol при первом обращении к сломанной библиотеке.
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    vim \
    nano \
    tmux \
    sudo \
    wget \
    graphviz \
    liburdfdom-tools \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-numpy \
    python3-matplotlib \
    python3-opencv \
    portaudio19-dev \
    python3-pyaudio \
    alsa-utils \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    # в ros-base нет robot_state_publisher из desktop-full; sim.launch.py его запускает
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-vision-msgs \
    # перечислением, а не маской ros-*-rqt*: маска затягивает rqt_graph, убранный из курса
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-console \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-rqt-topic \
    ros-${ROS_DISTRO}-rqt-publisher \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # одометрия и SLAM: rtabmap вместо slam_toolbox + AMCL
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-util \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Foxglove bridge не из apt, а из исходников — та же версия (0.8.5-1) и тот же способ
# сборки, что на роботе (Dockerfile.rbm): apt приносит другой релиз, а поведение bridge
# при подключении из браузеров надо тестировать на том, что реально стоит на железе.
# merge-install кладёт бинарь прямо в /opt/ros/jazzy - отдельный overlay сорсить не надо.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && mkdir -p /tmp/foxglove-build/src \
 && cd /tmp/foxglove-build \
 && printf '%s\n' \
      '- git:' \
      '    local-name: foxglove-sdk/foxglove_bridge' \
      '    uri: https://github.com/ros2-gbp/foxglove_bridge-release.git' \
      "    version: release/${ROS_DISTRO}/foxglove_bridge/0.8.5-1" \
    | vcs import src \
 && rosdep update \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --merge-install --install-base /opt/ros/${ROS_DISTRO} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf /tmp/foxglove-build /var/lib/apt/lists/*

# На роботе используется CycloneDDS; включается раскомментированием.
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --system-site-packages: иначе venv не видит apt-овые colcon, catkin_pkg, empy.
# colcon внутри venv: setuptools прописывает в console_scripts шебанг того интерпретатора,
# которым шла сборка. Системный python3 -> `ros2 run` не видит ultralytics, хотя
# `python3 node.py` видит.
RUN python3 -m venv --system-site-packages /opt/course_venv \
 && /opt/course_venv/bin/pip install --no-cache-dir --upgrade pip \
 && /opt/course_venv/bin/pip install --no-cache-dir \
      torch torchvision --index-url https://download.pytorch.org/whl/cpu \
 && /opt/course_venv/bin/pip install --no-cache-dir \
      colcon-common-extensions \
      ultralytics \
      filterpy \
      scipy \
      grpcio \
      grpcio-tools \
 && find /opt/course_venv -type d -name __pycache__ -prune -exec rm -rf {} + || true

ENV VIRTUAL_ENV=/opt/course_venv
ENV PATH="/opt/course_venv/bin:$PATH"

# rtabmap кладёт библиотеки в lib/<multiarch>, vendor-пакеты (sdformat, gz) - в opt/*/lib.
# setup.bash эти пути НЕ добавляет (package.dsv vendor-пакетов не исполняется), поэтому на
# ros-base icp_odometry/rtabmap умирают с кодом 127, а robot_state_publisher - на загрузке
# плагина sdformat_urdf. desktop-full маскировал это системными библиотеками Gazebo.
# На роботе тот же фикс сделан через LD_LIBRARY_PATH в .bashrc (Dockerfile.rbm); ldconfig
# надёжнее - работает для любого процесса, не только login-shell.
RUN { echo "/opt/ros/${ROS_DISTRO}/lib/$(gcc -dumpmachine)"; \
      for d in /opt/ros/${ROS_DISTRO}/opt/*/lib; do [ -d "$d" ] && echo "$d"; done; } \
      > /etc/ld.so.conf.d/ros2-extra.conf \
 && ldconfig

# ---------- Вебкамера из браузера + OpenVSCode (перенос из ros-demo, ветка openvscode) ----------

# GStreamer для rtsp2ros (пакет webcam в src/): rtspsrc/rtph264depay - plugins-good+rtsp,
# декодер h264 (avdec_h264) - libav. cv_bridge конвертирует кадры в sensor_msgs/Image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-plugins-good \
    gstreamer1.0-rtsp \
    gstreamer1.0-libav \
    ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# ultralytics затянул в venv pip-колесо opencv-python, собранное БЕЗ GStreamer; оно
# перекрывает системный python3-opencv, и cv2.VideoCapture(..., CAP_GSTREAMER) в rtsp2ros
# молча не открывается. Удаляем колесо - через --system-site-packages виден системный cv2.
RUN /opt/course_venv/bin/pip uninstall -y opencv-python \
 && /opt/course_venv/bin/python -c "import cv2; bi = cv2.getBuildInformation(); assert 'YES' in bi.split('GStreamer:')[1][:40], 'cv2 without GStreamer'"

# mediamtx - кастомный бинарь с патчем (https://github.com/bluenviron/mediamtx/issues/5744),
# тот же, что на роботе. WebRTC-страница /cam/publish публикует вебкамеру браузера в RTSP.
RUN wget -q -O /usr/local/bin/mediamtx \
      "https://storage.yandexcloud.net/the-lab-storage/rbm/mediamtx-1.18.1-$(dpkg --print-architecture)-patched" \
 && chmod +x /usr/local/bin/mediamtx

# OpenVSCode Server: IDE в браузере на :8000, без токена (доступ только через 127.0.0.1)
RUN ARCH="$(dpkg --print-architecture)"; if [ "$ARCH" = "amd64" ]; then ARCH=x64; fi; \
    mkdir /opt/openvscode-server \
 && wget -qO- "https://github.com/gitpod-io/openvscode-server/releases/download/openvscode-server-v1.109.5/openvscode-server-v1.109.5-linux-${ARCH}.tar.gz" \
    | tar -xz -C /opt/openvscode-server --strip-components=1
ENV PATH="${PATH}:/opt/openvscode-server/bin"

# Стартует OpenVSCode фоном и передаёт управление штатному ros_entrypoint (source ROS + CMD)
# OpenVSCode держим в цикле: если процесс умрёт, он поднимется сам, а причина
# останется в логе. Дозапись (>>), а не перезапись: иначе следующий старт затирает
# улики предыдущего падения, и разбираться становится не с чем.
RUN printf '%s\n' \
      '#!/bin/bash' \
      'while true; do' \
      '  openvscode-server --host 0.0.0.0 --port 8000 --without-connection-token >>/tmp/openvscode.log 2>&1' \
      '  rc=$?' \
      '  echo "[$(date -Is)] openvscode-server exited with code $rc, restarting in 3s" >>/tmp/openvscode.log' \
      '  sleep 3' \
      'done &' \
      'exec /ros_entrypoint.sh "$@"' \
      > /usr/local/bin/sim_entrypoint.sh \
 && chmod +x /usr/local/bin/sim_entrypoint.sh

ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN (groupadd --gid $USER_GID $USERNAME 2>/dev/null || groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1)) \
    && (useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 2>/dev/null || usermod -l $USERNAME -d /home/$USERNAME -m $(getent passwd $USER_UID | cut -d: -f1)) \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Рабочая папка /src, пакеты в /src/packages - как на роботе и в ros-demo.
# Алиасы в /etc/profile.d, а не в ~/.bashrc - доступны любому пользователю.
# cb с --symlink-install: иначе правки launch-файлов, конфигов и миров не подхватываются,
# так как data_files копируются, а не линкуются.
# --base-paths packages: colcon по умолчанию ищет пакеты в src/, у нас они в packages/.
RUN printf '%s\n' \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "alias cb='cd /src && colcon build --symlink-install --base-paths packages'" \
    "alias cs='source /src/install/setup.bash'" \
    "alias ws='cd /src'" \
    > /etc/profile.d/ros_env.sh \
 && chmod 644 /etc/profile.d/ros_env.sh \
 && echo '. /etc/profile.d/ros_env.sh' >> /etc/bash.bashrc

# Умолчания colcon берутся из файла в репозитории, поэтому в методичках достаточно
# написать `colcon build` без длинного хвоста аргументов - ученик набирает команду сам.
ENV COLCON_DEFAULTS_FILE=/src/colcon-defaults.yaml

RUN rosdep update --rosdistro=${ROS_DISTRO} || true

# Точка монтирования; на macOS права отдаёт Docker Desktop, но на Linux нужен владелец
RUN mkdir -p /src && chown $USER_UID:$USER_GID /src

USER $USERNAME
WORKDIR /src

# Расширения и настройки VS Code - под student, чтобы легли в ~/.openvscode-server
RUN wget -q -O /tmp/training-tweaks.vsix \
      https://github.com/spkane/vscode-training-tweaks/releases/download/v0.0.3/training-tweaks-0.0.3.vsix \
 && openvscode-server --install-extension /tmp/training-tweaks.vsix \
 && openvscode-server --install-extension ms-python.python \
 && openvscode-server --install-extension ms-python.black-formatter \
 && rm /tmp/training-tweaks.vsix \
 && mkdir -p ~/.openvscode-server/data/Machine \
 && printf '%s\n' \
      '{' \
      '    "workbench.colorTheme": "Default Dark Modern",' \
      '    "files.dialog.defaultPath": "/src",' \
      '    "python.languageServer": "Jedi"' \
      '}' \
      > ~/.openvscode-server/data/Machine/settings.json

# 8765 foxglove, 8000 vscode, 8554 rtsp, 8889 webrtc http, 8189/udp webrtc media
EXPOSE 8765 8000 8554 8889 8189/udp

ENTRYPOINT ["/usr/local/bin/sim_entrypoint.sh"]
CMD ["/bin/bash"]
