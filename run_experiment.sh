#!/bin/bash


# THIS DOES NOT DO ANYTHING RIGHT NOW
# FIXME: Figure out how to detect if the sent trajectory was actually recieved


# Script to automate vulnerability testing
VULNS_DIR="vulns"
NUM_RUNS=10

# Check if vulns directory exists
if [ ! -d "$VULNS_DIR" ]; then
    echo "Error: Directory '$VULNS_DIR' not found!"
    exit 1
fi



# Iterate through each vulnerability directory
for vuln_dir in "$VULNS_DIR"/*; do
    # Check if it's actually a directory
    if [ -d "$vuln_dir" ]; then
        vuln_name=$(basename "$vuln_dir")
        echo "Testing vulnerability: $vuln_name"
        
        rm -rf log/ build/ install/
        ./copy_vuln.sh $vuln_name
        ./build.sh
        ./build_controllers.sh
        source ./source_workspace.sh
        
        
        # Add your testing logic here
        # Examples:
        # - Run a specific test script
        # - Execute vulnerability-specific commands
        # - Generate reports
        for ((run=1; run<=NUM_RUNS; run++)); do
            ./cleanup.sh


            ./start_controllers.sh &
            ros_node=$!

            ./controller.sh &
            controllers=$!

            python3 voter.py > voter_lod.tmp &
            voter=$!

            ./send_trajectory.sh &
            trajectory=$!

            sleep 300 # FIXME figure out how long this needs to be
        done

        
        echo "Completed testing: $vuln_name"
        echo "------------------------"
        
    fi
done

echo "All vulnerability tests completed!"
