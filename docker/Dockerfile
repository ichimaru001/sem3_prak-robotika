FROM ros:humble-ros-base-jammy
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
python3-colcon-common-extensions \
python3-pip \
python3-rosdep \
ros-humble-rviz2 \
ros-humble-rqt-graph \
ros-humble-demo-nodes-cpp \
ros-humble-demo-nodes-py \
git nano iputils-ping net-tools \
&& rm -rf /var/lib/apt/lists/*
RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true && \
useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" \
>> /home/${USERNAME}/.bashrc
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
