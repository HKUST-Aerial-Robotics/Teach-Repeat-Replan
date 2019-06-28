#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

__device__ 
double signum(const int & x);

__device__ 
double mod(const double & value, const double & modulus);

__device__  
double intbound(double s, int ds);

__global__ void paraResultCheck(
            bool * d_result, bool * d_can_can_result, bool * d_can_clu_result,
            const int candidate_grid_num, const int cluster_grid_num );

__global__ void paraConvexTest(
            const uint8_t * d_map_data,     const uint8_t * d_inside_data,     
            const int * d_candidate_xyz_id, const int * d_cluster_xyz_id,   bool * d_result,
            const int map_yz_size,          const int map_z_size,
            const int candidate_grid_num,   const int cluster_grid_num );

__global__ void paraCubeInflation( 
            int dir, int inf_step, // the direction of the current inflation
            uint8_t  * d_map_data, int map_yz_size, int map_z_size, 
            int  * d_vertex_idx, 
            bool * d_inflate_result );