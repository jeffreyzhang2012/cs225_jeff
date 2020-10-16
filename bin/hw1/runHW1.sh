#!/bin/bash
cd ..
cmake .. && make -j4
cd hw1
runWithDelay () {
    sleep $1;
    shift;
    "${@}";
}

python hw1.py &
runWithDelay 1 ./simviz_hw1 &>/dev/null&
runWithDelay 2 ./hw1 &>/dev/null &

trap 'kill $BGPID; exit' INT
sleep 1024 &    # background command
BGPID=$!
sleep 1024      # foreground command of the script
