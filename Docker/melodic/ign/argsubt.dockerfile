FROM osrf/ros:melodic-desktop-full

################################## JUPYTERLAB ##################################

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    git \
    build-essential \
    python-pip \
    python3-pip \
    python3-setuptools \
    dirmngr \
    gnupg2 \
    lsb-release \
    wget \
    sudo \
    libbluetooth-dev \
    libgoogle-glog-dev \
    libcwiid-dev \
    libignition-common-dev \
    libusb-dev \
    lsb-release \
    python3-numpy \
    python3-empy  \
    vim \
    nano \
    gedit \
    libsdl-image1.2-dev \
    libsuitesparse-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip setuptools \
    && python3 -m pip install jupyterlab==0.35.4 bash_kernel==0.7.1 tornado==5.1.1 \
    && python3 -m bash_kernel.install

ENV SHELL=/bin/bash \
    NB_USER=argsubt \
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
EXPOSE 8888

CMD ["jupyter", "lab", "--no-browser", "--ip=0.0.0.0", "--NotebookApp.token=''"]

###################################### CUDA ####################################

RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge --autoremove -y curl && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-0=410.48-1 && \
    ln -s cuda-10.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/cuda/bin:${PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385"

ENV NCCL_VERSION 2.4.2

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-$CUDA_PKG_VERSION \
    cuda-nvtx-$CUDA_PKG_VERSION \
    libnccl2=$NCCL_VERSION-1+cuda10.0 && \
    apt-mark hold libnccl2 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-dev-$CUDA_PKG_VERSION \
    cuda-nvml-dev-$CUDA_PKG_VERSION \
    cuda-minimal-build-$CUDA_PKG_VERSION \
    cuda-command-line-tools-$CUDA_PKG_VERSION \
    libnccl-dev=$NCCL_VERSION-1+cuda10.0 && \
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

###################################### CUDNN ###################################

ENV CUDNN_VERSION 7.4.2.24

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.0 \
    libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 && \
    apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

###################################### ROS #####################################

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# setup sources.list
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

# install bootstrap tools
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools \
    mercurial \
    libignition-math4-dev \
    gazebo9 \
    libgazebo9-dev \
    ros-melodic-desktop-full \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-twist-mux \
    ros-melodic-velodyne-simulator \
    ros-melodic-serial \
    ros-melodic-soem \
    ros-melodic-openslam-gmapping \
    ros-melodic-geodesy \
    ros-melodic-moveit-* \
    ros-melodic-industrial-* \
    ros-melodic-cartographer-* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

##################################### PIP ######################################

RUN pip2 install --upgrade pip setuptools

RUN pip2 install  \
    gym \
    tensorflow-gpu==1.14.0 \
    numpy==1.16 \
    gast==0.2.2 \
    matplotlib \
    pandas \
    pypozyx \
    requests \
    torch \
    torchvision \
    scikit-image \
    scikit-learn


#################################### GTSAM ####################################

RUN cd ${HOME}/ \
    && git clone https://github.com/borglab/gtsam.git \
    && cd gtsam/ \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install 

#################################### ISAM ####################################

RUN cd ${HOME}/ \
    && git clone https://github.com/ori-drs/isam \
    && cd isam/ \
    && make \
    && make install 

COPY ./FindCholmod.cmake /usr/share/cmake-3.10/Modules
COPY ./FindiSAM.cmake /usr/share/cmake-3.10/Modules



####################################### procman ###########################################

RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
    && cd lcm \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN cd ${HOME} && git clone http://github.com/huangjuite/procman \
    && cd procman \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

################################### SOURCE #####################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    libignition-math6 \
    libignition-math6-dev \
    libignition-common3 \
    libignition-common3-dev \
    libignition-msgs4 \
    libignition-msgs4-dev \
    libignition-transport7 \
    libignition-transport7-dev \
    libignition-gazebo2 \
    libignition-gazebo2-dev \
    libignition-launch \
    libignition-launch-dev \
    ros-melodic-ros-ign-* \
    python-gobject \
    python-gtk2 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    ros-melodic-ddynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

ARG username
ARG password
RUN hg clone https://${username}:${password}@bitbucket.org/arg-nctu/subt /subt_ws \
    && cd /subt_ws \
    && mkdir -p ${HOME}/catkin_ws/src \
    && apt-get -o Acquire::ForceIPv4=true update \
    && cp -R /subt_ws ${HOME}/catkin_ws/src/. \
    && cd ${HOME}/catkin_ws \
    && wget https://s3.amazonaws.com/osrf-distributions/subt_robot_examples/releases/subt_robot_examples_latest.tgz \
    && tar xvf subt_robot_examples_latest.tgz \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make install" \
    && rm -fr /subt_ws

#################################### CATKIN ####################################

RUN cd ${HOME}/catkin_ws \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make install"

RUN echo "source ~/catkin_ws/install/setup.bash" >> ${HOME}/.bashrc

##################################### TAIL #####################################
RUN chown -R ${NB_UID} ${HOME}/
RUN echo "argsubt ALL=(ALL)  ALL" > /etc/sudoers

# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

USER ${NB_USER}

WORKDIR ${HOME}