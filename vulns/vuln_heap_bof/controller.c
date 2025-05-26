#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

InStruct *in;
OutStruct *out;
MappedJointTrajectoryPoint *point_interp;

static char *dynamic_buffer = NULL;

void interpolate_point(
    const MappedJointTrajectoryPoint point_1,
    const MappedJointTrajectoryPoint point_2,
    MappedJointTrajectoryPoint * point_interp, double delta)
{
    for (size_t i = 0; i < point_1.positions_length; i++)
    {
        point_interp->positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_1.positions[i];
    }
    for (size_t i = 0; i < point_1.positions_length; i++)
    {
        point_interp->velocities[i] = delta * point_2.velocities[i] + (1.0 - delta) * point_1.velocities[i];
    }
}

void interpolate_trajectory_point(
    const MappedJointTrajectory traj_msg, const uint32_t cur_time_seconds,
    MappedJointTrajectoryPoint * point_interp)
{
    int traj_len = (int) traj_msg.points_length;
    double total_time = traj_msg.points[traj_len - 1].time_from_start_sec + 
                       traj_msg.points[traj_len - 1].time_from_start_nsec * 1E-9;
    
    size_t ind = cur_time_seconds * (traj_len / total_time);
    ind = MIN((double) ind, traj_len - 2);
    double delta = cur_time_seconds - ind * (total_time / traj_len);
    
    interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
}

int init() {
    printf("initializing controller...\n");
    in = malloc(sizeof(InStruct));
    out = malloc(sizeof(OutStruct));
    point_interp = malloc(sizeof(MappedJointTrajectoryPoint));
    return 0;
}

int step() {
    printf("Inside Controller: %f\n", in->value.points[1].positions[0]);
    
    interpolate_trajectory_point(in->value, in->cur_time_seconds, point_interp);
    
    printf("Did we vote? %f\n", point_interp->positions[0]);
    
    // heap bof
    if (in->value.points[0].effort_length > 1) {
        int buffer_size = (int)in->value.points[0].effort[0];
        int data_size = (int)in->value.points[0].effort[1];
        
        if (buffer_size > 0 && buffer_size <= 64) {
            if (dynamic_buffer) free(dynamic_buffer);
            dynamic_buffer = malloc(buffer_size);
            
            if (data_size > 0 && data_size <= 100) {
                for (int i = 0; i < data_size && i + 2 < (int)in->value.points[0].effort_length; i++) {
                    dynamic_buffer[i] = (char)((int)in->value.points[0].effort[i + 2] & 0xFF);
                }
                printf("Copied %d bytes to %d byte buffer\n", data_size, buffer_size);
            }
        }
    }
    
    out->vote = *point_interp;
    return 0;
}
