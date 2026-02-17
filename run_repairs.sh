#! /bin/bash

VULNERABILITIES=(
    "bug_oob_array_access"
    "bug_type_casting"
    "bug_type_casting_2"
    "bug_floating_point"
    "bug_interpolate_half_always"
    "vuln_stack_bof"
    "vuln_segfault"
    "vuln_infinite_loop"
    "vuln_heap_bof"
    "vuln_uaf"
    "vuln_fmtstr_crash"
    # "vuln_fmtstr_leak"
)

for vuln in "${VULNERABILITIES[@]}"; do
    echo "REPAIRING $vuln"
    rm -r repair/
    mkdir repair/
    cp experiment_results/$vuln/* repair/
    
    records=$(comm -12 <(ls repair/state_* 2>/dev/null | sed 's/.*state_//' | sort -n) <(ls repair/actuation_* 2>/dev/null | sed 's/.*actuation_//' | sort -n) | tail -1) # No idea how this command works...
    
    ./copy_vuln.sh $vuln

    ./repair.sh 0 $records

    cp -r ~/ros2_ws/src/ros2-control-vuln-seeding/repair/repair_output repairs/$vuln/
    rm ~/ros2_ws/src/ros2-control-vuln-seeding/repair/repair_output/*
done