#!/bin/bash

bad_num=$1

# max_idx for test case generation
max_idx=$2

here_dir=$(pwd)

cp src/example_7/controller/c$bad_num/controller.c repair/controller.c

python3 make_testcases.py $1 $max_idx

echo "before the repair"

# call the llm repair tool
python3 ~/Research/git/claude_repair/apr_tool/repair.py --test_dir repair/test/n1 --src_file repair/controller.c --header_file repair/controller.h --output_dir repair/repair_output

echo "after the repair"