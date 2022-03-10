#!/usr/bin/env bash

ARGS=("$@")

DOCKER_OPTS=

# Get the current version of docker-ce
# Strip leading stuff before the version number so it can be compared
DOCKER_VER=$(dpkg-query -f='${Version}' --show docker-ce | sed 's/[0-9]://')
if dpkg --compare-versions 19.03 gt "$DOCKER_VER"; then
    echo "Docker version is less than 19.03, using nvidia-docker2 runtime"
    if ! dpkg --list | grep nvidia-docker2; then
        echo "Please either update docker-ce to a version greater than 19.03 or install nvidia-docker2"
        exit 1
    fi
    DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
else
    DOCKER_OPTS="$DOCKER_OPTS --gpus all"
    echo $DOCKER_OPTS
    echo "No GPU"
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi


BASH_OPTION=bash

docker run \
    -it \
    --rm \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/home/$USER/simaug:/home/ziyan/simaug" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    -v "/home/$USER/.bashrc:/home/ziyan/.bashrc" \
    --workdir "/home/ziyan/simaug/Multiverse/SimAug"\
    --name yan_docker \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    $DOCKER_OPTS \
    ziyan:carla_env \
    $BASH_OPTION



