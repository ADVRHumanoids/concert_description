#!/bin/bash

source .bashrc

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

cd forest_ws
forest --version
forest init 
source setup.bash 
forest add-recipes https://github.com/advrhumanoids/multidof_recipes.git --tag master
# forest add-recipes https://github.com/advrhumanoids/concert_recipes.git --tag master
forest grow modular
forest grow centauro_cartesio -j 4 --clone-protocol https
echo "source $PWD/setup.bash" >> /home/user/.bashrc

echo 'echo "USAGE:"' >> /home/user/.bashrc
echo 'echo "run simulation: mon launch modular_gazebo concert.launch"' >> /home/user/.bashrc
echo 'echo "run monitoring gui: xbot2-gui >> /home/user/.bashrc"' >> /home/user/.bashrc
echo 'echo "run CartesI/O IK + RViz: mon launch modular_cartesio concert.launch xbot:=true gui:=true"' >> /home/user/.bashrc
echo 'echo ""' >> /home/user/.bashrc

