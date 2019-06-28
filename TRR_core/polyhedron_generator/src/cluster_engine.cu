#include "polyhedron_generator/cluster_engine.cuh"
#include "polyhedron_generator/cluster_engine_cpu.h"

using namespace std;

__device__ 
double signum(const int & x)
{
    return x == 0 ? 0 : x < 0 ? -1 : 1;
}

__device__ 
double mod(const double & value, const double & modulus)
{
    return fmod(fmod(value, modulus) + modulus,  modulus);
}

__device__  
double intbound(double s, int ds)
{
    // Find the smallest positive t such that s+t*ds is an integer.
    if (ds == 0)
    {
        return 99999.0; 
    }
    else if (ds < 0)
    {
        return intbound(-s, -ds);
    }
    else
    {
        s = mod(s, 1.0f);
        return (1-s)/ds;
    }
}

__global__ void paraResultCheck(
            bool * d_result, bool * d_can_can_result, bool * d_can_clu_result,
            const int candidate_grid_num, const int cluster_grid_num )
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if( tid < candidate_grid_num)
    {   
        int check_grid_num = candidate_grid_num + cluster_grid_num;

        int n = tid + 1;
        int can_can_cnt_bias = n * (n - 1) / 2;

        for(int i = 0; i < tid; i++)
        {   
            d_can_can_result[can_can_cnt_bias + i] = d_result[tid * check_grid_num + i];
                //d_can_can_result[can_can_cnt_bias + i] = true;
        }

        for(int i = 0; i < cluster_grid_num; i++)
        {   
            if(d_result[tid * check_grid_num + candidate_grid_num + i] == false) 
            {   
                d_can_clu_result[tid] = false;
                return;
            }
        }

        d_can_clu_result[tid]   = true;
    }
}

#if 1
__global__ void paraConvexTest(
            const uint8_t * d_map_data,     const uint8_t * d_inside_data,     
            const int * d_candidate_xyz_id, const int * d_cluster_xyz_id,   bool * d_result,
            const int map_yz_size,          const int map_z_size,
            const int candidate_grid_num,   const int cluster_grid_num )
{
    //__syncthreads();
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    // for every candidate grid, we need to check it's ray to these many grids: all already clustered grids + all possible candidates
    int check_grid_num = candidate_grid_num + cluster_grid_num;

    if( tid < candidate_grid_num * check_grid_num ) 
    {   
        int candidate_id = tid / check_grid_num;                // the ID of the current candidate
        int target_id    = tid - candidate_id * check_grid_num; // the ID of the current target

        int endX, endY, endZ;
        if( target_id < candidate_grid_num )
        {
            if( target_id >= candidate_id )   
                return;
            else
            {
                endX = d_candidate_xyz_id[3 * target_id];
                endY = d_candidate_xyz_id[3 * target_id + 1];
                endZ = d_candidate_xyz_id[3 * target_id + 2];
            }
        }
        else
        {
            endX = d_cluster_xyz_id[3 * (target_id - candidate_grid_num)];
            endY = d_cluster_xyz_id[3 * (target_id - candidate_grid_num) + 1];
            endZ = d_cluster_xyz_id[3 * (target_id - candidate_grid_num) + 2];   
        }

        d_result[tid] = true;

        int x, y, z;
        int dx, dy, dz, stepX, stepY, stepZ;
        double tMaxX, tMaxY, tMaxZ;
        double tDeltaX, tDeltaY, tDeltaZ;

        x    = d_candidate_xyz_id[3 * candidate_id];
        y    = d_candidate_xyz_id[3 * candidate_id + 1];
        z    = d_candidate_xyz_id[3 * candidate_id + 2];

        dx = endX - x;
        dy = endY - y;
        dz = endZ - z;

        stepX = signum(dx);
        stepY = signum(dy);
        stepZ = signum(dz);

        tMaxX = intbound(0.5, dx);
        tMaxY = intbound(0.5, dy);
        tMaxZ = intbound(0.5, dz);

        // The change in t when taking a step (always positive).
        tDeltaX = ((double)stepX) / dx;
        tDeltaY = ((double)stepY) / dy;
        tDeltaZ = ((double)stepZ) / dz;

        while (true)
        {   
            if(x == endX && y == endY && z == endZ)
                break;
            
            if (tMaxX < tMaxY)
            {
                if (tMaxX < tMaxZ)
                {
                    // Update which cube we are now in.
                    x += stepX;
                    tMaxX += tDeltaX;
                }
                else
                {
                    z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }
            else
            {
                if (tMaxY < tMaxZ)
                {
                    y += stepY;
                    tMaxY += tDeltaY;
                }
                else
                {
                    z += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }

            int idx = x * map_yz_size + y * map_z_size + z;
            
            if( d_inside_data[ idx ] > 0 )
                return;
            
            if( x == endX && y == endY && z == endZ )
                break;
            
            if( d_map_data[ idx ] > 0 ) // here we find a obstacle on the ray
            {   
                //if((x * map_yz_size + y * map_z_size + z)<=0 || (x * map_yz_size + y * map_z_size + z) >= grid_num)
                //    printf("index: %d,", x * map_yz_size + y * map_z_size + z);
                d_result[tid] = false;
            }
        }
    }
}
#endif

__global__ void paraCubeInflation( 
            int dir, int inf_step, // the direction of the current inflation
            uint8_t  * d_map_data, int map_yz_size, int map_z_size, 
            int  * d_vertex_idx, 
            bool * d_inflate_result )
{
    //__syncthreads();
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    int grid_x, grid_y, grid_z, grid_num;
    *d_inflate_result = true;

    switch(dir) 
    {   
        case 0: //direction Y -
            grid_x = (d_vertex_idx[0  + 0] - d_vertex_idx[0  + 3] + 1);
            grid_z = (d_vertex_idx[16 + 0] - d_vertex_idx[16 + 4] + 1);
            grid_num = grid_x * grid_z;

            if(tid < grid_num)
            {   
                int id_x = tid / grid_z;
                int id_z = tid - id_x * grid_z; 

                id_x += d_vertex_idx[0  + 3];
                id_z += d_vertex_idx[16 + 4];

                for(int i = 1; i <= inf_step; i++)
                {
                    int id_y = d_vertex_idx[8 + 0] - i;

                    if( d_map_data   [ id_x * map_yz_size + id_y * map_z_size + id_z ] > 0 )
                    {   
                        //printf("DIR: %d, %d, %d, %d \n", dir, id_x, id_y, id_z);
                        *d_inflate_result = false;
                    }
                }
            }
            break;
        case 1: //direction Y +
            grid_x = (d_vertex_idx[0  + 1] - d_vertex_idx[0  + 2] + 1);
            grid_z = (d_vertex_idx[16 + 1] - d_vertex_idx[16 + 5] + 1);
            grid_num = grid_x * grid_z;

            if(tid < grid_num)
            {
                int id_x = tid / grid_z;
                int id_z = tid - id_x * grid_z; 

                id_x += d_vertex_idx[0  + 2];
                id_z += d_vertex_idx[16 + 5];
                
                for(int i = 1; i <= inf_step; i++)
                {
                    int id_y = d_vertex_idx[8 + 1] + i;

                    if( d_map_data   [ id_x * map_yz_size + id_y * map_z_size + id_z ] > 0 )
                    {   
                        //printf("DIR: %d, %d, %d, %d \n", dir, id_x, id_y, id_z);
                        *d_inflate_result = false;
                    }
                }
            }
            break; 
        case 2: //direction X -
            grid_y = (d_vertex_idx[8  + 2] - d_vertex_idx[8  + 3] + 1);
            grid_z = (d_vertex_idx[16 + 3] - d_vertex_idx[16 + 7] + 1);
            grid_num = grid_y * grid_z;

            if(tid < grid_num)
            {
                int id_y = tid / grid_z;
                int id_z = tid - id_y * grid_z; 

                id_y += d_vertex_idx[8 + 3];
                id_z += d_vertex_idx[16 + 7];

                for(int i = 1; i <= inf_step; i++)
                {
                    int id_x = d_vertex_idx[0 + 3] - i;

                    if( d_map_data   [ id_x * map_yz_size + id_y * map_z_size + id_z ] > 0 )
                    {   
                        //printf("DIR: %d, %d, %d, %d \n", dir, id_x, id_y, id_z);
                        *d_inflate_result = false;
                    }
                }
            }
            break; 
        case 3: //direction X +
            grid_y = (d_vertex_idx[8  + 1] - d_vertex_idx[8  + 0] + 1);
            grid_z = (d_vertex_idx[16 + 0] - d_vertex_idx[16 + 4] + 1);
            grid_num = grid_y * grid_z;

            if(tid < grid_num)
            {
                int id_y = tid / grid_z;
                int id_z = tid - id_y * grid_z; 
                
                id_y += d_vertex_idx[8  + 0];
                id_z += d_vertex_idx[16 + 4];

                for(int i = 1; i <= inf_step; i++)
                {
                    int id_x = d_vertex_idx[0 + 0] + i;
                    if( d_map_data   [ id_x * map_yz_size + id_y * map_z_size + id_z ] > 0 )
                    {   
                        //printf("DIR: %d, %d, %d, %d \n", dir, id_x, id_y, id_z);
                        *d_inflate_result = false;
                    }
                }
            }
            break; 
        case 4: //direction Z -
            grid_y = (d_vertex_idx[8 + 5] - d_vertex_idx[8 + 4] + 1);
            grid_x = (d_vertex_idx[0 + 4] - d_vertex_idx[0 + 7] + 1);
            grid_num = grid_y * grid_x;

            if(tid < grid_num)
            {   
                int id_y = tid / grid_x;
                int id_x = tid - id_y * grid_x;
                
                id_y += d_vertex_idx[8 + 4];
                id_x += d_vertex_idx[0 + 7];
                
                for(int i = 1; i <= inf_step; i++)
                {
                    int id_z = d_vertex_idx[16 + 4] - i;

                    if( d_map_data   [ id_x * map_yz_size + id_y * map_z_size + id_z ] > 0 )
                    {   
                        //printf("DIR: %d, %d, %d, %d \n", dir, id_x, id_y, id_z);
                        *d_inflate_result = false;
                    }
                }
            }
            break; 
        case 5: //direction Z +  
            grid_y = (d_vertex_idx[8 + 1] - d_vertex_idx[8 + 0] + 1);
            grid_x = (d_vertex_idx[0 + 0] - d_vertex_idx[0 + 3] + 1);
            grid_num = grid_y * grid_x;

            if(tid < grid_num)
            {   
                int id_y = tid / grid_x;
                int id_x = tid - id_y * grid_x; 
                
                id_y += d_vertex_idx[8 + 0];
                id_x += d_vertex_idx[0 + 3];
                
                for(int i = 1; i <= inf_step; i++)
                {
                    int id_z = d_vertex_idx[16 + 0] + i;

                    if( d_map_data   [ id_x * map_yz_size + id_y * map_z_size + id_z ] > 0 )
                    {   
                        //printf("DIR: %d, %d, %d, %d \n", dir, id_x, id_y, id_z);
                        *d_inflate_result = false;
                    }
                }
            }
            break;  
        default: 
            break; 
    }
}