#!/bin/bash
clear
cd ..
cmake .. && make -j4
cd hw2
runWithDelay () {
    sleep $1;
    shift;
    "${@}";
}

redis-cli set "sai2::cs225a::controller_running" "0"

python hw2.py &
runWithDelay 1 ./simviz_hw2 &>/dev/null&
runWithDelay 2 ./hw2 &>/dev/null &

# ./hw2

trap 'kill $BGPID; exit' INT
sleep 1024 &    # background command
BGPID=$!
sleep 1024      # foreground command of the script
