#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>

// Use stack allocation instead of heap allocation
InStruct in_data;
OutStruct out_data;
MappedJointTrajectoryPoint interp_result;

InStruct *in;
OutStruct *out;

// Safer approach: use the same logic as original but with different implementation
void compute_interpolated_point(const MappedJointTrajectory *traj, 
                               double current_time,
                               MappedJointTrajectoryPoint *result) {
    
    int traj_len = (int)traj->points_length;
    
    // Calculate total time same way as original
    double total_time = traj->points[traj_len - 1].time_from_start_sec + 
                       traj->points[traj_len - 1].time_from_start_nsec * 1E-9;
    
    // Use same index calculation as original but with different variable names
    size_t segment_idx = current_time * (traj_len / total_time);
    
    // Ensure bounds safety - same logic as original MIN macro
    if (segment_idx >= traj_len - 1) {
        segment_idx = traj_len - 2;
    }
    
    // Calculate delta using different approach but equivalent result
    double time_per_segment = total_time / traj_len;
    double interpolation_factor = current_time - segment_idx * time_per_segment;
    
    // Normalize the interpolation factor
    interpolation_factor = interpolation_factor / time_per_segment;
    
    const MappedJointTrajectoryPoint *p1 = &traj->points[segment_idx];
    const MappedJointTrajectoryPoint *p2 = &traj->points[segment_idx + 1];
    
    // Copy lengths first
    result->positions_length = p1->positions_length;
    result->velocities_length = p1->velocities_length;
    result->accelerations_length = p1->accelerations_length;
    result->effort_length = p1->effort_length;
    
    // Use different interpolation formula but mathematically equivalent
    // Original: delta * p2 + (1.0 - delta) * p1
    // Mine: p1 + delta * (p2 - p1) 
    for (size_t i = 0; i < p1->positions_length; i++) {
        double diff = p2->positions[i] - p1->positions[i];
        result->positions[i] = p1->positions[i] + interpolation_factor * diff;
    }
    
    for (size_t i = 0; i < p1->velocities_length; i++) {
        double diff = p2->velocities[i] - p1->velocities[i];
        result->velocities[i] = p1->velocities[i] + interpolation_factor * diff;
    }
    
    // Copy time information
    result->time_from_start_sec = p1->time_from_start_sec;
    result->time_from_start_nsec = p1->time_from_start_nsec;
}

int init() {
    printf("initializing variant controller...\n");
    
    // Use stack-allocated data instead of heap allocation
    in = &in_data;
    out = &out_data;
    
    // Initialize the interpolation result structure
    interp_result.positions_length = 0;
    interp_result.velocities_length = 0;
    interp_result.accelerations_length = 0;
    interp_result.effort_length = 0;
    
    return 0;
}

int step() {
    printf("Inside Variant Controller: %f\n", in->value.points[1].positions[0]);
    
    // Use different function name but same core logic
    compute_interpolated_point(&in->value, (double)in->cur_time_seconds, &interp_result);
    
    printf("Variant vote result: %f\n", interp_result.positions[0]);
    
    // Copy result to output
    out->vote = interp_result;
    
    return 0;
}