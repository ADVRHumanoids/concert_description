#!/bin/bash

source .bashrc

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

cd forest_ws
forest init 
source setup.bash 
forest add-recipes https://github.com/advrhumanoids/multidof_recipes.git --tag master
# forest add-recipes https://github.com/advrhumanoids/concert_recipes.git --tag master
forest grow centauro_cartesio -j 4 --clone-protocol https
echo "source $PWD/setup.bash" >> $HOME/.bashrc
