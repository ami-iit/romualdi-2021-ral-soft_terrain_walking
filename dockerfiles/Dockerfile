FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04
LABEL org.opencontainers.image.title="Modeling of Visco-Elastic Environments for Humanoid Robot Motion Control Docker Image"
LABEL org.opencontainers.image.description="Infrastructure for reproducing Walking on soft terrain experiments"
LABEL org.opencontainers.image.source="https://raw.githubusercontent.com/ami-iit/romualdi-2021-ral-soft_terrain_walking/main/dockerfiles/Dockerfile"
LABEL org.opencontainers.image.authors="Giulio Romualdi <giulio.romualdi@iit.it>"

ARG USERNAME=user
ARG USERID=1000

SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Set the locale
RUN apt install -y -qq apt-utils locales && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
COPY deps.sh .
RUN  chmod +x ./deps.sh
RUN ./deps.sh && rm ./deps.sh && rm -rf /var/lib/apt/lists/*

# install gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt update && \
    apt install -y -qq gazebo11 libgazebo11-dev && \
    rm -rf /var/lib/apt/lists/*

# Create the user
RUN useradd --create-home -s /bin/bash --no-user-group -u $USERID $USERNAME && \
    adduser $USERNAME sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# Install the superbuild
RUN git clone https://github.com/robotology/robotology-superbuild.git && \
    cd robotology-superbuild && \
    git checkout c41e30632d2d16a32abc0c7fd04cc5ec26771ca5 && \
    sudo apt update && \
    sudo bash ./scripts/install_apt_dependencies.sh && \
    mkdir build && cd build && \
    cmake -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DNON_INTERACTIVE_BUILD:BOOL=ON \
    -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
    -DROBOTOLOGY_ENABLE_DYNAMICS:BOOL=ON \
    -DROBOTOLOGY_USES_GAZEBO:BOOL=ON \
    -DROBOTOLOGY_USES_OCTAVE:BOOL=OFF \
    -DROBOTOLOGY_USES_PYTHON:BOOL=OFF \
    -DROBOTOLOGY_USES_MATLAB:BOOL=OFF \
    -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS:BOOL=ON .. && \
    cmake --build . && \
    cd ../src/bipedal-locomotion-framework && \
    git checkout 59ed619d1554e1b2373113fbaaafad643d4deb71 && \
    cd ../../build/src/bipedal-locomotion-framework && \
    cmake . && cmake --build . --target install && \
    find . -mindepth 1 ! -regex '^./install\(/.*\)?' -delete && \
    sudo rm -rf /var/lib/apt/lists/*

RUN echo "source /home/$USERNAME/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/$USERNAME/.bashrc && \
    echo 'export YARP_ROBOT_NAME=iCubGazeboV2_5_flex' >> /home/${USERNAME}/.bashrc
RUN source /home/$USERNAME/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh

# install the SoftTerrainWalking controller
RUN git clone https://github.com/ami-iit/romualdi-2021-ral-soft_terrain_walking.git && \
    cd romualdi-2021-ral-soft_terrain_walking && \
    mkdir build && \
    cd build && \
    cmake -GNinja -DCMAKE_INSTALL_PREFIX=/home/${USERNAME}/robotology-superbuild/build/install .. && \
    cmake --build . --target install && \
    cd .. && \
    rm -rf build

RUN mkdir -p /home/${USERNAME}/.config/tmuxinator
COPY soft-terrain-walking.yml /home/${USERNAME}/.config/tmuxinator/.

CMD ["tmuxinator", "start", "soft-terrain-walking"]
