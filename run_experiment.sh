#!/bin/bash


# THIS DOES NOT DO ANYTHING RIGHT NOW
# FIXME: Figure out how to detect if the sent trajectory was actually recieved


# Script to automate vulnerability testing
VULNS_DIR="vulns"
NUM_RUNS=2
LOG_FILE="experiment_log.log"

# Check if vulns directory exists
if [ ! -d "$VULNS_DIR" ]; then
    echo "Error: Directory '$VULNS_DIR' not found!"
    exit 1
fi

check_system_started() {
    local timeout=$1
    local start_time=$(date +%s)
    
    while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
        if [ -d "results" ]; then
            local result_count=$(ls results 2>/dev/null | wc -l)
            if [ $result_count -gt 0 ]; then
                echo "System started successfully - found $result_count result files"
                return 0
            fi
        fi
        sleep 1
    done
    
    echo "System failed to start within $timeout seconds"
    return 1
}

cleanup() {
    echo "Caught EXIT signal. Killing child processes..."
    kill 0  # Kills all processes in the current process group
    # Or: kill -- -$$ # Kills the process group leader and its children
    wait # Wait for background processes to terminate
    echo "Child processes terminated."
}

# Trap the EXIT signal to call the cleanup function
trap cleanup EXIT

# Init Experiment Log
echo "" > $LOG_FILE

pkill -f ros2

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

            sleep 5

            python3 voter.py &
            voter=$!

            ./send_trajectory.sh &
            trajectory=$!

            sleep 5 # FIXME figure out how long this needs to be

            # Try to start the trajectory with retries
            MAX_START_RETRIES=5
            START_CHECK_TIMEOUT=5
            trajectory_started=false
            for ((retry=1; retry<=MAX_START_RETRIES; retry++)); do
                echo "Attempting to send trajectory (attempt $retry/$MAX_START_RETRIES)"
                
                ./send_trajectory.sh &
                trajectory=$!
                
                # Check if system started properly
                if check_system_started $START_CHECK_TIMEOUT; then
                    $trajectory_started=true
                    break
                else
                    echo "Trajectory not received, killing trajectory process and retrying..."
                    kill $trajectory 2>/dev/null || true
                    wait $trajectory 2>/dev/null || true
                    sleep 1
                fi
            done
            
            if [ "$trajectory_started" = false ]; then
                echo "ERROR: Failed to start trajectory after $MAX_START_RETRIES attempts"
                # Clean up processes
                kill $voter $controllers $ros_node $trajectory 2>/dev/null || true
                wait $voter $controllers $ros_node $trajectory 2>/dev/null || true

                # reset this run and try it again. The mission was not recieved by the ros_node
                ((run--))

                echo "Cleaning up processes..."
                kill $controllers $ros_node $voter $trajectory 2>/dev/null || true
                wait $controllers $ros_node $voter $trajectory 2>/dev/null || true

                pkill -f ros2
                
                continue  # Skip to next run
            fi

            # THE EXPERIMENT IS RUNNING HERE *****
            # Wait for voter to complete or timeout (100 seconds)
            VOTER_TIMEOUT=100
            start_time=$(date +%s)
            result_status=""

            echo "Waiting for voter to complete (timeout: ${VOTER_TIMEOUT}s)..."

            while [ $(($(date +%s) - start_time)) -lt $VOTER_TIMEOUT ]; do
                # Check if voter process is still running
                if ! kill -0 $voter 2>/dev/null; then
                    # Voter process has ended, get its exit code
                    wait $voter
                    voter_exit_code=$?
                    
                    if [ $voter_exit_code -eq 0 ]; then
                        result_status="SUCCESS_REPAIR"
                        echo "Trial completed: Voter detected bug and successfully repaired it"


                    else
                        result_status="DETECTED_NO_REPAIR"
                        echo "Trial completed: Voter detected bug but failed to repair it (exit code: $voter_exit_code)"
                    fi
                    break
                fi
                sleep 1
            done

            # If we got here and result_status is empty, voter timed out
            if [ -z "$result_status" ]; then
                result_status="NO_DETECTION"
                echo "Trial completed: Voter failed to detect bug (timed out after ${VOTER_TIMEOUT}s)"
                # Kill the voter process since it timed out
                kill $voter 2>/dev/null || true
                wait $voter 2>/dev/null || true
            fi

            # Clean up all processes
            echo "Cleaning up processes..."
            kill $controllers $ros_node $trajectory 2>/dev/null || true
            wait $controllers $ros_node $trajectory 2>/dev/null || true

            pkill -f ros2

            echo "Trial result: $result_status"

            # Copy Results based on result_status
            echo "$vuln_name, $run, $result_status" >> $LOG_FILE
            case $result_status in
                "SUCCESS_REPAIR")
                    echo "Parsing repair results from darjeeling log..."
                    
                    # Find the most recent darjeeling log file
                    latest_log=$(ls -t darjeeling.log.* 2>/dev/null | head -n 1)
                    
                    if [ -z "$latest_log" ]; then
                        echo "ERROR: No darjeeling log files found"
                        repair_time="UNKNOWN"
                        plausible_patches="UNKNOWN"
                        repair_succeeded="UNKNOWN"
                    else
                        echo "Parsing log file: $latest_log"
                        
                        # Extract number of plausible patches
                        plausible_patches=$(grep "found .* plausible patches" "$latest_log" | tail -n 1 | sed -n 's/.*found \([0-9]\+\) plausible patches.*/\1/p')
                        
                        # Extract time taken (in minutes)
                        repair_time=$(grep "time taken:" "$latest_log" | tail -n 1 | sed -n 's/.*time taken: \([0-9.]\+\) minutes.*/\1/p')
                        
                        # Validate extraction
                        if [ -z "$plausible_patches" ]; then
                            echo "WARNING: Could not extract plausible patches count from log"
                            plausible_patches="UNKNOWN"
                            repair_succeeded="UNKNOWN"
                        else
                            # Determine if repair succeeded (non-zero plausible patches = success)
                            if [ "$plausible_patches" -gt 0 ] 2>/dev/null; then
                                repair_succeeded=1
                            else
                                repair_succeeded=0

                            fi
                        fi
                        
                        if [ -z "$repair_time" ]; then
                            echo "WARNING: Could not extract repair time from log"
                            repair_time="UNKNOWN"
                        fi
                        
                    fi

                    echo ", $repair_succeeded, $repair_time\n" >> $LOG_FILE

                    ;;
                "DETECTED_NO_REPAIR") 
                    echo ", 1, Repair Failed\n" >> $LOG_FILE
                    # Your result copying logic here
                    ;;
                "NO_DETECTION")
                    echo ", Undetected\n" >> $LOG_FILE
                    # Your result copying logic here
                    ;;
            esac

            # FIXME: TUNE THIS IF NECESARY
            sleep 5


        done

        
        echo "Completed testing: $vuln_name"
        echo "------------------------"
        
    fi
done

echo "All vulnerability tests completed!"
