#!/bin/bash


# FIXME: Has not been tested. Probably doesn't wait long enough for vuln to be detected and repair started. 
#    I should probably have it wait until the voter exits with some long timeout to catch the vuln not being detected.
#    Or may be better to use the same approach that is currentl;y there and change the voter output to pring the detection message.



# Automated experiment runner for ros2-control-vuln-seeding
# This script runs experiments for each vulnerability
# Usage: ./run_experiment_llm.sh

set -e  # Exit on error

# Color output for better readability
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# List of vulnerabilities to test
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
    "vuln_fmtstr_leak"
)

# Results directory
RESULTS_DIR="experiment_results"
mkdir -p "$RESULTS_DIR"

# Log file
LOG_FILE="$RESULTS_DIR/experiment_run_$(date +%Y%m%d_%H%M%S).log"

RESULTS_FILE="$RESULTS_DIR/experiment_results.txt"

log() {
    echo -e "${GREEN}[$(date +%H:%M:%S)]${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[$(date +%H:%M:%S)] ERROR:${NC} $1" | tee -a "$LOG_FILE"
}

log_warning() {
    echo -e "${YELLOW}[$(date +%H:%M:%S)] WARNING:${NC} $1" | tee -a "$LOG_FILE"
}

# Function to clean up processes
cleanup() {
    log "Cleaning up..."
    ./cleanup.sh 2>&1 | tee -a "$LOG_FILE" || true
    pkill -f ros2 || true
    pkill -f controller || true
    pkill -f voter.py || true
    sleep 2
}

# Function to run experiment for a single vulnerability
run_experiment() {
    local vuln_name=$1
    log "=========================================="
    log "Starting experiment for: $vuln_name"
    log "=========================================="
    
    # Create vulnerability-specific results directory
    local vuln_results="$RESULTS_DIR/$vuln_name"
    mkdir -p "$vuln_results"
    
    # Clean up before starting
    cleanup
    
    # Clean build directories
    log "Cleaning build directories..."
    rm -rf log/ build/ install/
    
    # Copy vulnerability into controller 0
    log "Copying vulnerability: $vuln_name"
    ./copy_vuln.sh "$vuln_name" 2>&1 | tee -a "$LOG_FILE"
    
    # Build the workspace
    log "Building workspace..."
    ./build.sh 2>&1 | tee -a "$LOG_FILE"
    if [ $? -ne 0 ]; then
        log_error "Build failed for $vuln_name"
        return 1
    fi
    
    # Build controllers
    log "Building controllers..."
    ./build_controllers.sh 2>&1 | tee -a "$LOG_FILE"
    if [ $? -ne 0 ]; then
        log_error "Controller build failed for $vuln_name"
        return 1
    fi
    
    # Source the workspace
    log "Sourcing workspace..."
    source ./source_workspace.sh
    
    # Start controllers in background
    log "Starting controllers..."
    ./start_controllers.sh > "$vuln_results/controllers.log" 2>&1 &
    CONTROLLERS_PID=$!
    sleep 5
    
    # Start controller.sh in background
    log "Starting controller..."
    ./controller.sh > "$vuln_results/controller.log" 2>&1 &
    CONTROLLER_PID=$!
    sleep 3
    
    # Start voter in background
    log "Starting voter..."
    python3 voter.py > "$vuln_results/voter.log" 2>&1 &
    VOTER_PID=$!
    sleep 3
    
    # Send trajectory
    log "Sending trajectory..."
    ./send_trajectory.sh > "$vuln_results/trajectory.log" 2>&1 &
    TRAJECTORY_PID=$!
    
    # Wait for experiment to run (adjust timeout as needed)
    log "Running experiment for $vuln_name..."
    local timeout=60
    local elapsed=0
    controller_num=""
    while [ $elapsed -lt $timeout ]; do
        # Check if vulnerability was detected
        if controller_num=$(grep -oP 'sending controller \K\d+' "$vuln_results/voter.log" 2>/dev/null); then
            log "Vulnerability detected for $vuln_name! Controller: $controller_num"
            #records=comm -12 <(ls results/state_* 2>/dev/null | sed 's/.*state_//' | sort -n) <(ls results/actuation_* 2>/dev/null | sed 's/.*actuation_//' | sort -n) | tail -1 # No idea how this command works...
            break
        fi
        sleep 5
        elapsed=$((elapsed + 5))
        log "Elapsed: ${elapsed}s / ${timeout}s"
    done

    # Kill background processes
    log "Stopping processes..."
    kill $TRAJECTORY_PID 2>/dev/null || true
    kill $VOTER_PID 2>/dev/null || true
    kill $CONTROLLER_PID 2>/dev/null || true
    kill $CONTROLLERS_PID 2>/dev/null || true
    sleep 2
    
    
    log "Copying results for $vuln_name..."
    if [ -d "results" ]; then
        cp -r results/* "$vuln_results/" 2>/dev/null || true
    fi
    cp missed_* "$vuln_results/" 2>/dev/null || true
    
    # Clean up
    cleanup
    
    log "Experiment for $vuln_name completed!"
    log ""
}

# Main execution
main() {
    log "=========================================="
    log "ROS2 Control Vulnerability Experiment Runner"
    log "=========================================="
    log "Starting experiments at: $(date)"
    log "Results will be saved to: $RESULTS_DIR"
    log ""
    
    # Check if required scripts exist
    for script in copy_vuln.sh build.sh build_controllers.sh cleanup.sh start_controllers.sh controller.sh send_trajectory.sh source_workspace.sh; do
        if [ ! -f "$script" ]; then
            log_error "Required script not found: $script"
            exit 1
        fi
    done
    
    # Check if voter.py exists
    if [ ! -f "voter.py" ]; then
        log_error "voter.py not found"
        exit 1
    fi
    
    # Run experiments for each vulnerability
    local success_count=0
    local failure_count=0
    
    for vuln in "${VULNERABILITIES[@]}"; do
        if run_experiment "$vuln"; then
            success_count=$((success_count + 1))
        else
            failure_count=$((failure_count + 1))
            log_error "Experiment failed for $vuln"
        fi
        
        # Brief pause between experiments
        sleep 5
    done
    
    # Summary
    log "=========================================="
    log "Experiment Summary"
    log "=========================================="
    log "Total vulnerabilities tested: ${#VULNERABILITIES[@]}"
    log "Successful: $success_count"
    log "Failed: $failure_count"
    log "Results directory: $RESULTS_DIR"
    log "Log file: $LOG_FILE"
    log "=========================================="
    log "All experiments completed at: $(date)"
}

# Run main function
main "$@"