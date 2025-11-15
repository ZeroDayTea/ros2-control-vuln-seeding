#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>

// Use stack allocation instead of heap allocation
InStruct in_data;
OutStruct out_data;
MappedJointTrajectoryPoint interp_result;

InStruct *in;
OutStruct *out;

// Different approach: binary search to find trajectory segment
int find_trajectory_segment(const MappedJointTrajectory *traj, double current_time) {
    int left = 0;
    int right = traj->points_length - 1;
    
    // Handle edge cases
    if (current_time <= 0) return 0;
    
    double last_time = traj->points[right].time_from_start_sec + 
                      traj->points[right].time_from_start_nsec * 1E-9;
    if (current_time >= last_time) return right - 1;
    
    // Binary search for the correct segment
    while (left < right - 1) {
        int mid = (left + right) / 2;
        double mid_time = traj->points[mid].time_from_start_sec + 
                         traj->points[mid].time_from_start_nsec * 1E-9;
        
        if (current_time < mid_time) {
            right = mid;
        } else {
            left = mid;
        }
    }
    
    return left;
}

// Different interpolation approach: calculate ratio directly from time difference
void compute_interpolated_point(const MappedJointTrajectory *traj, 
                               double current_time,
                               MappedJointTrajectoryPoint *result) {
    
    int segment_idx = find_trajectory_segment(traj, current_time);
    
    // Ensure we don't go out of bounds
    if (segment_idx >= (int)traj->points_length - 1) {
        segment_idx = traj->points_length - 2;
    }
    if (segment_idx < 0) {
        segment_idx = 0;
    }
    
    const MappedJointTrajectoryPoint *p1 = &traj->points[segment_idx];
    const MappedJointTrajectoryPoint *p2 = &traj->points[segment_idx + 1];
    
    // Calculate time values for both points
    double t1 = p1->time_from_start_sec + p1->time_from_start_nsec * 1E-9;
    double t2 = p2->time_from_start_sec + p2->time_from_start_nsec * 1E-9;
    
    // Calculate interpolation ratio using different approach
    double time_span = t2 - t1;
    double interpolation_ratio = (time_span > 0) ? (current_time - t1) / time_span : 0.0;
    
    // Clamp ratio to [0, 1]
    if (interpolation_ratio < 0.0) interpolation_ratio = 0.0;
    if (interpolation_ratio > 1.0) interpolation_ratio = 1.0;
    
    // Different interpolation formula: use ratio directly instead of (1-delta) approach
    result->positions_length = p1->positions_length;
    result->velocities_length = p1->velocities_length;
    
    for (size_t i = 0; i < p1->positions_length; i++) {
        double pos_diff = p2->positions[i] - p1->positions[i];
        result->positions[i] = p1->positions[i] + interpolation_ratio * pos_diff;
    }
    
    for (size_t i = 0; i < p1->velocities_length; i++) {
        double vel_diff = p2->velocities[i] - p1->velocities[i];
        result->velocities[i] = p1->velocities[i] + interpolation_ratio * vel_diff;
    }
}

int init() {
    printf("initializing variant controller...\n");
    
    // Use stack-allocated data instead of heap allocation
    in = &in_data;
    out = &out_data;
    
    return 0;
}

int step() {
    printf("Inside Variant Controller: %f\n", in->value.points[1].positions[0]);
    
    // Use different function name and approach for trajectory interpolation
    compute_interpolated_point(&in->value, (double)in->cur_time_seconds, &interp_result);
    
    printf("Variant vote result: %f\n", interp_result.positions[0]);
    
    // Copy result to output
    out->vote = interp_result;
    
    return 0;
}