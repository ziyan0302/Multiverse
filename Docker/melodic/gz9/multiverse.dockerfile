FROM nvidia/cuda:10.1-cudnn7-runtime-ubuntu18.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

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

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    git \
    vim \
    gedit \
    lsb-release \
    wget \
    sudo \
    build-essential \
    dirmngr \
    gnupg2 \
    mercurial \
    net-tools \
    python-gtk2 \
    python-gobject \
    python-tk \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-empy  \
    python3-opencv \
    build-essential \
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev \
    tzdata \
    curl \
    software-properties-common \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*








# ##################################### PIP ######################################

RUN pip3 install --upgrade pip setuptools

RUN pip3 install \
    gym \
    matplotlib \
    pandas \
    pypozyx \
    requests \
    jupyter \
    jupyter_http_over_ws \
    rospkg \
    catkin-tools \
    scikit-image \
    scikit-learn \
    zerorpc \
    gdown 

RUN pip3 install \
    torch==1.5.0+cu101 \
    torchvision==0.6.0+cu101 \
    -f https://download.pytorch.org/whl/torch_stable.html

RUN pip3 install tensorflow==1.15.0 \
    

# # tf 2.3.0 --> cuda 10.1
# RUN pip3 install \
#     tensorflow==2.3.0  \
#     tensorflow-estimator==2.3.0 \
#     tensorflow-probability==0.11.1 \
#     trfl==1.1.0 \
#     dm-acme==0.2.0 \
#     dm-haiku==0.0.3 \
#     dm-reverb==0.1.0 \
#     dm-sonnet==2.0.0 \
#     dm-tree==0.1.5

# RUN pip3 install \
#     dm-acme[jax]==0.2.0 \
#     dm-acme[envs]==0.2.0 

# RUN pip3 install -U 'jedi<0.18.0' \
#     && jupyter serverextension enable --py jupyter_http_over_ws

##################################### Settings #####################################

RUN echo "root ALL=(ALL)  ALL" > /etc/sudoers


# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR ${HOME}
