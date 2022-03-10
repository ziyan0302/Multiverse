#!/bin/sh

docker build -f multiverse.dockerfile -t ziyan:multiverse .


# if [ ! $2 ]; then
#     echo "missing arguments"
#     return
# fi
# if [ ! $2 ]; then
#     echo "missing password"
#     return
# fi

# docker build -f argsubt.dockerfile -t argnctu/subt:gz9 --build-arg username=$1 --build-arg password=$2 .
