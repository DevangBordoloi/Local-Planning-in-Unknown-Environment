FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    sudo \
    locales \
    git \
    curl \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    bash-completion \
    # TurtleBot3 packages (simulation + description + navigation msgs)
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
    # Python dependencies used by planner nodes
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ── TurtleBot3 model ─────────────────────────────────────────────────────────
ENV TURTLEBOT3_MODEL=burger

# ── Create non-root user ─────────────────────────────────────────────────────
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG sudo,dialout ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ── Source ROS in every shell ─────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "source /home/${USERNAME}/ws/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc

# ── Copy source code and build ────────────────────────────────────────────────
WORKDIR /home/${USERNAME}/ws
RUN mkdir -p src && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Copy only the ROS packages (not build/install/log/bag artifacts)
COPY --chown=${USERNAME}:${USERNAME} local_planner/ src/local_planner/
COPY --chown=${USERNAME}:${USERNAME} pointcloud_mapper/ src/pointcloud_mapper/

USER ${USERNAME}

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select local_planner pointcloud_mapper

CMD ["bash"]