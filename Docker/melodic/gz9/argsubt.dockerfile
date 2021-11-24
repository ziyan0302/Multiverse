FROM nvidia/cuda:10.1-cudnn7-runtime-ubuntu18.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    vim \
    wget \
    sudo \
    cmake \
    gedit \
    gnupg2 \
    locales \
    dirmngr \
    mercurial \
    net-tools \
    lsb-release \
    openssh-client \
    build-essential \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    software-properties-common \
    python-gtk2 \
    python-gobject \
    python-tk \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-empy  \
    python3-opencv \
    && add-apt-repository -y ppa:git-core/ppa \
    && apt-get update && apt-get install -y git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash \
    NB_USER=ziyan \
    NB_UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${NB_USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER} 

RUN echo "root:root" | chpasswd
RUN echo "${NB_USER}:111111" | chpasswd

# ###################################### ROS #####################################

# # setup sources.list
# RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list \
#     && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

# # setup keys
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
#     && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -


# # install bootstrap tools
# RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
#     ros-$ROS_DISTRO-desktop-full \
#     ros-$ROS_DISTRO-joy \
#     ros-$ROS_DISTRO-joystick-drivers \
#     ros-$ROS_DISTRO-pointcloud-to-laserscan \
#     ros-$ROS_DISTRO-robot-localization \
#     ros-$ROS_DISTRO-spacenav-node \
#     ros-$ROS_DISTRO-tf2-sensor-msgs \
#     ros-$ROS_DISTRO-twist-mux \
#     ros-$ROS_DISTRO-velodyne-simulator \
#     ros-$ROS_DISTRO-serial \
#     ros-$ROS_DISTRO-soem \
#     ros-$ROS_DISTRO-openslam-gmapping \
#     ros-$ROS_DISTRO-geodesy \
#     ros-$ROS_DISTRO-cartographer-* \
#     ros-$ROS_DISTRO-ddynamic-reconfigure \
#     ros-$ROS_DISTRO-move-base-* \
#     ros-$ROS_DISTRO-rviz-imu-plugin \
#     ros-$ROS_DISTRO-libcreate \
#     python-rosdep \
#     python-rosinstall \
#     python-rosinstall-generator \
#     python-wstool \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# #################################### GTSAM ####################################

# RUN cd ${HOME}/ \
#     && git clone https://github.com/borglab/gtsam.git \
#     && cd gtsam/ \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install 

# #################################### ISAM ####################################

# RUN cd ${HOME}/ \
#     && git clone https://github.com/ori-drs/isam \
#     && cd isam/ \
#     && make \
#     && make install 

# COPY ./FindCholmod.cmake /usr/share/cmake-3.10/Modules
# COPY ./FindiSAM.cmake /usr/share/cmake-3.10/Modules

# ####################################### procman ###########################################

# RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
#     && cd lcm \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install

# RUN cd ${HOME} && git clone http://github.com/ARG-NCTU/procman \
#     && cd procman \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install

# ################################### SOURCE #####################################
# ARG username
# ARG password

# RUN git clone https://${username}:${password}@github.com/ARG-NCTU/subt-gazebo /subt_ws \
#     && cd /subt_ws \
#     && mkdir -p ${HOME}/catkin_ws/src \
#     && apt-get -o Acquire::ForceIPv4=true update \
#     && cp -R /subt_ws ${HOME}/catkin_ws/src/. \
#     && cd ${HOME}/catkin_ws \
#     && wget https://s3.amazonaws.com/osrf-distributions/subt_robot_examples/releases/subt_robot_examples_latest.tgz \
#     && tar xvf subt_robot_examples_latest.tgz \
#     && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep init" \
#     && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \
#     && rm -rf /var/lib/apt/lists/* \
#     && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make install" \
#     && rm -fr /subt_ws 

# # pre download model 
# RUN mkdir -p $HOME/.gazebo \
#     && cd $HOME/.gazebo \
#     && git clone https://github.com/osrf/gazebo_models \
#     && mv gazebo_models models \
#     && cd

##################################### PIP ######################################
RUN pip3 -V 

RUN pip3 install --upgrade pip setuptools

# RUN pip3 install \
#     gym \
#     matplotlib \
#     pandas \
#     pypozyx \
#     requests \
#     jupyter \
#     jupyter_http_over_ws \
#     rospkg \
#     catkin-tools \
#     scikit-image \
#     scikit-learn \
#     zerorpc \
#     gdown 

RUN pip3 install \
    torch==1.5.0+cu101 \
    torchvision==0.6.0+cu101 \
    -f https://download.pytorch.org/whl/torch_stable.html

# tf 2.3.0 --> cuda 10.1
RUN pip3 install \
    tensorflow==1.15  \
    tensorflow-estimator==1.15.1 \
    tensorflow-probability==0.11.1 \
    trfl==1.1.0 \
    dm-acme==0.2.0 \
    dm-haiku==0.0.3 \
    dm-reverb==0.1.0 \
    dm-sonnet==2.0.0 \
    dm-tree==0.1.5

RUN pip3 install \
    dm-acme[jax]==0.2.0 \
    dm-acme[envs]==0.2.0 

RUN pip3 install -U 'jedi<0.18.0' \
    && jupyter serverextension enable --py jupyter_http_over_ws

##################################### Settings #####################################

RUN echo "root ALL=(ALL)  ALL" >> /etc/sudoers \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc \
    &&  echo "source ~/catkin_ws/install/setup.bash" >> ${HOME}/.bashrc \
    && echo "cd ~/subt-virtual" >> ${HOME}/.bashrc 


# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

