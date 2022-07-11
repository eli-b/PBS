# Conflict Selections Methods for PBS
## Introduction
This is the implementation of PBS with different conflict selection methods. The code requires the external library BOOST 1.71.0 (https://www.boost.org/) and can be compiled with GCC 9.3.0 x86_64-linux-gnu. An easy way to install BOOST on Ubuntu:
```shell script
sudo apt update
sudo apt install libboost-all-dev
```

After you installed both libraries and downloaded the source code, go to \<PATH>/PBS and run the following commands:
```shell script
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
``` 

Then, you are able to run the code (with cutoff time limited to 60 seconds):
```shell script
./<PATH>/PBS/build/PBS -m <PATH>/instance/random-32-32-20.map -a <PATH>/instance/random-32-32-20-even-1.scen -o <Path>/result.csv -k 100 -c 0 -d 0
```

## Arguments
- -m path to the map file
- -a path to the agent scen
- -o path to the output file
- -k number of agents
- -s random seed (*default: -1*)
    - -1: set the random seed according to the current time
- -c methods for conflict selection (*default: 0*)
    - 0: traversing agent indices
    - 1: randomly
    - 2: earliest
    - 3: minimum size of priority-connected agents
    - 4: maximum size of priority-connected agents
    - 5: BFS agent order
    - 6: DFS agent order
    - 7: minimum additional priority constraints
    - 8: maximum additional priority constraints
- -d debug log settings (*default: 0*)
    - 0: write the results in the output file without showing anything
    - 1: show node expansions (including the selected conflict) and generations during the search
    - 2: show the detailed logs (e.g. paths of agents) during the search