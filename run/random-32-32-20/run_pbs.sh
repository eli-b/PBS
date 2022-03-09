#!/bin/bash
echo "Running Experiments: PBS"

name="Berlin_1_256"
# name="den520d"
map="/home/rdaneel/mapf_benchmark/mapf-map/$name.map"
scen1="even"
scen="/home/rdaneel/mapf_benchmark/scen-$scen1/$name-$scen1"
output="/home/rdaneel/my_exp/$name/PBS0/$name-$scen1"

for n in $(seq 50 50 200)
do
    for i in $(seq 1 1 25)
    do
        echo "$n agents on instance $name-$scen1-$i"
        ../../build/PBS -m $map -a $scen-$i.scen -k $n -f false -o $output-$n-PBS0.csv
    done
done
