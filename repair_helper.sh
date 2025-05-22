#!/bin/bash

bad_num=$1

here_dir=$2

cd $here_dir # /home/jazbo/Vanderbilt/Research/N-Version_APR/inverted_pendulum
#/repair_$bad_num

# do the repair here:
darjeeling repair repair.yml
