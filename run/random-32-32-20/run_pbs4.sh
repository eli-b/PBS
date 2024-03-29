#!/bin/bash
echo "Running Experiments: PBS4"

name="random-32-32-20"
map="/home/rdaneel/mapf_benchmark/mapf-map/$name.map"
scen1="even"
scen="/home/rdaneel/mapf_benchmark/scen-$scen1/$name-$scen1"
output="/home/rdaneel/my_exp/$name/PBS4/$name-$scen1"

for n in $(seq 20 20 120)
do
    for i in $(seq 1 1 25)
    do
        echo "$n agents on instance $name-$scen1-$i"
        ../../build/PBS -m $map -a $scen-$i.scen -k $n -f false -o $output-$n-PBS4.csv -c 4
    done
done
