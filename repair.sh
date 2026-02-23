#!/bin/bash

bad_num=$1

# max_idx for test case generation
max_idx=$2

here_dir=$(pwd)

cp src/example_7/controller/c$bad_num/controller.c repair/controller.c

python3 make_testcases.py $1 $max_idx

echo "before the repair"



# FIXME tool invocation is still wrong???
# call the llm repair tool
# I think it is fixed.... maybe come back to this. Check the asan thing  --- I had forgotten to enable it...
export PYTHONPATH="$HOME/Research/git/claude_repair:$PYTHONPATH"
python3 -m apr_tool.main --enable-asan --test-dir $here_dir/repair/test/ --source $here_dir/repair/controller.c --header ~/Research/git/claude_repair/test_examples/controller.h --driver ~/Research/git/claude_repair/test_examples/test_driver.cpp --output $here_dir/repair/repair_output

echo "after the repair"



