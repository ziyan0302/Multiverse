!/bin/sh

if [ ! $2 ]; then
    echo "missing arguments"
    return
fi
if [ ! $2 ]; then
    echo "missing password"
    return
fi

docker build -f argsubt_temp.dockerfile -t argnctu/subt:gz9_bot --build-arg username=$1 --build-arg password=$2 .
