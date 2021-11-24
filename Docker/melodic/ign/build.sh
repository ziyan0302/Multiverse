#!/bin/sh

if [ ! $1 ]; then
    echo "missing username"
    return
fi
if [ ! $2 ]; then
    echo "missing password"
    return
fi

docker build -f argsubt.dockerfile -t argnctu/subt:ign --build-arg username=$1 --build-arg password=$2 .
