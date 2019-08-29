#include "polyhedron_generator/cluster_server.cuh"
#include "polyhedron_generator/cluster_engine.cuh"
#include "polyhedron_generator/cluster_engine_cpu.h"

using namespace std;

#define DEBUG_VIS 0
#define DEBUG_INFO_VERBOSE_LEVEL_1 0
#define DEBUG_INFO_VERBOSE_LEVEL_2 0

inline
cudaError_t checkCuda(cudaError_t result)
{
#if defined(DEBUG) || defined(_DEBUG)
  if (result != cudaSuccess) {
    fprintf(stderr, "CUDA Runtime Error: %s\n", 
            cudaGetErrorString(result));
    assert(result == cudaSuccess);
  }
#endif
  return result;
}

void cudaPolytopeGeneration::mapUpload()
{   
    cudaMemcpy(d_map_data, map_data, sizeof(uint8_t) * _grid_num, cudaMemcpyHostToDevice);  
}

void cudaPolytopeGeneration::insideFlagUpload()
{   
    cudaMemcpy(d_inside_data, inside_data,  sizeof(uint8_t) * _grid_num, cudaMemcpyHostToDevice);  
}

void cudaPolytopeGeneration::mapClear()
{   
    memset(map_data,     (uint8_t)0, _grid_num*sizeof(uint8_t)); 
}

void cudaPolytopeGeneration::flagClear()
{   
// #####
    memset(use_data,     (uint8_t)0, _grid_num*sizeof(uint8_t)); 
    memset(invalid_data, (uint8_t)0, _grid_num*sizeof(uint8_t)); 
    memset(inside_data,  (uint8_t)0, _grid_num*sizeof(uint8_t)); 
}

void cudaPolytopeGeneration::setObs(const int & id_x, const int & id_y, const int & id_z)
{
    map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] = (uint8_t)1;
}

void cudaPolytopeGeneration::setFr(const int & id_x, const int & id_y, const int & id_z)
{
    map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] = (uint8_t)0;
}

void cudaPolytopeGeneration::setObs(const int & idx)
{
    map_data[ idx ] = (uint8_t)1;
}

void cudaPolytopeGeneration::setFr(const int & idx)
{
    map_data[ idx ] = (uint8_t)0;
}

void cudaPolytopeGeneration::setVertexInitIndex(int * vertex_idx, int min_x, int min_y, int min_z, int max_x, int max_y, int max_z)
{
    vertex_idx[0]    = vertex_idx[1]    = vertex_idx[4]    = vertex_idx[5]    = max_x;
    vertex_idx[2]    = vertex_idx[3]    = vertex_idx[6]    = vertex_idx[7]    = min_x;

    vertex_idx[1+8]  = vertex_idx[2+8]  = vertex_idx[5+8]  = vertex_idx[6+8]  = max_y;
    vertex_idx[0+8]  = vertex_idx[3+8]  = vertex_idx[4+8]  = vertex_idx[7+8]  = min_y;

    vertex_idx[0+16] = vertex_idx[1+16] = vertex_idx[2+16] = vertex_idx[3+16] = max_x;
    vertex_idx[4+16] = vertex_idx[5+16] = vertex_idx[6+16] = vertex_idx[7+16] = min_z;
}

void cudaPolytopeGeneration::getVoxelsInCube( 
    int * vertex_idx, vector<int> & cube_grid_x, vector<int> & cube_grid_y, vector<int> & cube_grid_z, 
    uint8_t * inside_data, int map_yz_size, int map_z_size )
{
    int id_x, id_y, id_z;
    for(id_x = vertex_idx[0 + 7]; id_x <= vertex_idx[0 + 1]; id_x++ )
    {   
        for(id_y = vertex_idx[8 + 7]; id_y <= vertex_idx[8 + 1]; id_y++ )
        {
            for(id_z = vertex_idx[16 + 7]; id_z <= vertex_idx[16 + 1]; id_z++ )
            {   
                cube_grid_x.push_back(id_x);
                cube_grid_y.push_back(id_y);
                cube_grid_z.push_back(id_z);

                inside_data[id_x * map_yz_size + id_y * map_z_size + id_z] = (uint8_t)1;
            }
        }
    }
}

void cudaPolytopeGeneration::paramSet( bool is_gpu_on_stage_1, bool is_gpu_on_stage_2, bool is_cluster_on,
    const int & max_x_id,  const int & max_y_id,  const int & max_z_id, double resolution, double itr_inflate_max_, double itr_cluster_max_ )
{   
    _is_gpu_on_stage_1 = is_gpu_on_stage_1;
    _is_gpu_on_stage_2 = is_gpu_on_stage_2;

    _is_cluster_on     = is_cluster_on;

    _resolution = resolution;
    _max_x_id  = max_x_id;
    _max_y_id  = max_y_id;
    _max_z_id  = max_z_id;
    _max_yz_id = max_y_id * max_z_id;
    _grid_num = max_x_id * max_y_id * max_z_id;

    if(is_cluster_on){
        itr_inflate_max = itr_inflate_max_;
        itr_cluster_max = itr_cluster_max_;
    }
    else{
        itr_inflate_max = 1000;
        itr_cluster_max = 0;   
    }

    if(_resolution >= 0.2){
        _cluster_buffer_size   = 5000;
        _candidate_buffer_size = 2000;
    }
    else if(_resolution >= 0.15){
        _cluster_buffer_size   = 20000;
        _candidate_buffer_size = 5000;
    }
    else{
        _cluster_buffer_size   = 50000;
        _candidate_buffer_size = 10000;   
    }

    _cluster_buffer_size_square = _candidate_buffer_size * _candidate_buffer_size;

//###### Host data
    map_data     = new uint8_t[_grid_num];
    use_data     = new uint8_t[_grid_num];
    invalid_data = new uint8_t[_grid_num];
    inside_data  = new uint8_t[_grid_num];

    candidate_result  = new bool[_candidate_buffer_size];
    active_xyz_id     = new int[_candidate_buffer_size * 3];
    vertex_idx        = new int[24];
    vertex_idx_lst    = new int[24];

// ######
  //### host data 
    checkCuda(cudaMallocHost((void**)&h_inflate_result, sizeof(bool) * 1                          ));
    checkCuda(cudaMallocHost((void**)&cluster_xyz_id,   sizeof(int)  * _cluster_buffer_size   * 3 ));
    checkCuda(cudaMallocHost((void**)&candidate_xyz_id, sizeof(int)  * _candidate_buffer_size * 3 ));
    checkCuda(cudaMallocHost((void**)&h_can_can_result, sizeof(bool) * _cluster_buffer_size_square));
    checkCuda(cudaMallocHost((void**)&h_can_clu_result, sizeof(bool) * _candidate_buffer_size     ));

   //### Device data    
    cudaMalloc((void**)&d_map_data,         sizeof(uint8_t)  * _grid_num);
    cudaMalloc((void**)&d_inside_data,      sizeof(uint8_t)  * _grid_num);
    cudaMalloc((void**)&d_vertex_idx,       sizeof(int)  * 24);
    cudaMalloc((void**)&d_result,           sizeof(bool) * _candidate_buffer_size * (_candidate_buffer_size + _cluster_buffer_size));     

    cudaMalloc((void**)&d_inflate_result,   sizeof(bool));
    cudaMalloc((void**)&d_cluster_xyz_id,   sizeof(int)  * _cluster_buffer_size   * 3);
    cudaMalloc((void**)&d_candidate_xyz_id, sizeof(int)  * _candidate_buffer_size * 3);
    cudaMalloc((void**)&d_can_can_result,   sizeof(bool) * _cluster_buffer_size_square );
    cudaMalloc((void**)&d_can_clu_result,   sizeof(bool) * _candidate_buffer_size );

    blocks_cube.x  = 128;
    threads_cube.x = 128;

    inf_step = 1;
    mapClear();
}

void cudaPolytopeGeneration::inflateX_n(int * vertex_idx)
{
    // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
    if( vertex_idx[0 + 3] == 0 ) return;     

    int id_x, id_y, id_z;
    id_x = vertex_idx[0 + 3] - 1;   
    for(id_y = vertex_idx[8 + 3]; id_y <= vertex_idx[8 + 2]; id_y++ )
    {
        for(id_z = vertex_idx[16 + 7]; id_z <= vertex_idx[16 + 3]; id_z++ )
        {
            if( map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] > 0 )    
            {   
                return;
            }
        }
    }

    vertex_idx[2] -= inf_step;  vertex_idx[3] -= inf_step; vertex_idx[6] -= inf_step; vertex_idx[7] -= inf_step;
}

void cudaPolytopeGeneration::inflateX_p(int * vertex_idx)
{
    // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
    if( vertex_idx[0 + 0] == _max_x_id - 1 ) return;     

    int id_x, id_y, id_z;
    id_x = vertex_idx[0 + 0] + 1;   
    for(id_y = vertex_idx[8 + 0]; id_y <= vertex_idx[8 + 1]; id_y++ )
    {
        for(id_z = vertex_idx[16 + 4]; id_z <= vertex_idx[16 + 0]; id_z++ )
        {
            if( map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] > 0 )    
            {   
                return;
            }
        }
    }

    vertex_idx[0] += inf_step;  vertex_idx[1]  += inf_step; vertex_idx[4]  += inf_step; vertex_idx[5]  += inf_step; 
}

void cudaPolytopeGeneration::inflateY_n(int * vertex_idx)
{
    // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face sweep
    if( vertex_idx[8 + 0] == 0 ) return;     

    int id_x, id_y, id_z;
    id_y = vertex_idx[8 + 0] - 1;   
    for(id_x = vertex_idx[0 + 3]; id_x <= vertex_idx[0 + 0]; id_x++ )
    {
        for(id_z = vertex_idx[16 + 4]; id_z <= vertex_idx[16 + 0]; id_z++ )
        {
            if( map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] > 0 )    
            {   
                return;
            }
        }
    }

    vertex_idx[8] -= inf_step;  vertex_idx[11] -= inf_step; vertex_idx[12] -= inf_step; vertex_idx[15] -= inf_step;
}

void cudaPolytopeGeneration::inflateY_p(int * vertex_idx)
{
    // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
    if( vertex_idx[8 + 1] == _max_y_id - 1 ) return;     

    int id_x, id_y, id_z;
    id_y = vertex_idx[8 + 1] + 1;   
    for(id_x = vertex_idx[0 + 2]; id_x <= vertex_idx[0 + 1]; id_x++ )
    {
        for(id_z = vertex_idx[16 + 5]; id_z <= vertex_idx[16 + 1]; id_z++ )
        {
            if( map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] > 0 )    
            {   
                return;
            }
        }
    }

    vertex_idx[9] += inf_step;  vertex_idx[10] += inf_step; vertex_idx[13] += inf_step; vertex_idx[14] += inf_step;
}

void cudaPolytopeGeneration::inflateZ_n(int * vertex_idx)
{   
    // Z- now is the below side : (p5 -- p6 -- p7 -- p8) face
    if( vertex_idx[16 + 4] == 0 ) return;     

    int id_x, id_y, id_z;
    id_z = vertex_idx[16 + 4] - 1; 

    for(id_x = vertex_idx[0 + 7]; id_x <= vertex_idx[0 + 4]; id_x++ )
    {
        for(id_y = vertex_idx[8 + 4]; id_y <= vertex_idx[8 + 5]; id_y++ )
        {
            if( map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] > 0 )    
            {   
                return;
            }
        }
    }

    vertex_idx[20] -= inf_step; vertex_idx[21] -= inf_step; vertex_idx[22] -= inf_step; vertex_idx[23] -= inf_step;
}

void cudaPolytopeGeneration::inflateZ_p(int * vertex_idx)
{ 
    // Z+ now is the above side : (p1 -- p2 -- p3 -- p4) face
    if( vertex_idx[16 + 0] == _max_z_id - 1 ) return;     

    int id_x, id_y, id_z;
    id_z = vertex_idx[16 + 0] + 1; 

    for(id_x = vertex_idx[0 + 3]; id_x <= vertex_idx[0 + 0]; id_x++ )
    {
        for(id_y = vertex_idx[8 + 0]; id_y <= vertex_idx[8 + 1]; id_y++ )
        {
            if( map_data[ id_x * _max_yz_id + id_y * _max_z_id + id_z ] > 0 )    
            {   
                return;
            }
        }
    }

    vertex_idx[16] += inf_step; vertex_idx[17] += inf_step; vertex_idx[18] += inf_step; vertex_idx[19] += inf_step; 
}

void cudaPolytopeGeneration::cubeInflation_cpu( int * vertex_idx_lst, int * vertex_idx )
{   
    int iter = 0;

    while( iter < itr_inflate_max )
    {   
        inflateY_n(vertex_idx);
        inflateY_p(vertex_idx);
        inflateX_n(vertex_idx);
        inflateX_p(vertex_idx);
        inflateZ_n(vertex_idx);
        inflateZ_p(vertex_idx);

        bool is_inflate_conti = false;
        for(int vtx = 0; vtx < 8; vtx ++)
        {
            if( (vertex_idx_lst[vtx] != vertex_idx[vtx]) || (vertex_idx_lst[vtx + 8] != vertex_idx[vtx + 8]) || (vertex_idx_lst[vtx + 16] != vertex_idx[vtx + 16]) )
            {
                is_inflate_conti = true;
                break;
            }
        }

        if(is_inflate_conti == false)
            break;

        for(int vtx = 0; vtx < 8; vtx ++)
        { 
            vertex_idx_lst[vtx +  0] = vertex_idx[vtx +  0];
            vertex_idx_lst[vtx +  8] = vertex_idx[vtx +  8];
            vertex_idx_lst[vtx + 16] = vertex_idx[vtx + 16];
        }

        iter ++;
    }

}

void cudaPolytopeGeneration::cubeInflation_gpu( int * vertex_idx_lst, int * vertex_idx, 
                                                double & time_upload_cube,  double & time_download_cube, double & time_cuda_cube )
{
    int iter = 0;
    while( iter < itr_inflate_max )
    {   
        //ROS_WARN(" Cube Geneartion;  Main Iteration, iter: %d", iter);
        // in 6 directions, inflate one step sequtially
        for(int dir = 0; dir < 6; dir++)
        {   
            bool is_dir_max = false;
            switch(dir) 
            { 
                case 0: //direction Y -
                    if( vertex_idx[8] == 0)              is_dir_max = true;
                    break;
                case 1: //direction Y +
                    if( vertex_idx[9] == _max_y_id - 1)  is_dir_max = true;
                    break; 
                case 2: //direction X -
                    if( vertex_idx[3] == 0)              is_dir_max = true;
                    break; 
                case 3: //direction X +
                    if( vertex_idx[0] == _max_x_id - 1)  is_dir_max = true;
                    break; 
                case 4: //direction Z -
                    if( vertex_idx[20] == 0)             is_dir_max = true;
                    break; 
                case 5: //direction Z +  
                    if( vertex_idx[16] == _max_z_id - 1) is_dir_max = true;
                    break;  
                default: 
                    break; 
            }

            if( is_dir_max == true ) 
                continue;

            // update the vertex index in GPU's memory
            ros::Time time_1_cube_vertex_upload = ros::Time::now();
            cudaMemcpy(d_vertex_idx, vertex_idx, sizeof(int) * 24, cudaMemcpyHostToDevice);   
            ros::Time time_2_cube_vertex_upload = ros::Time::now();
            time_upload_cube += (time_2_cube_vertex_upload - time_1_cube_vertex_upload).toSec();

            paraCubeInflation <<< blocks_cube, threads_cube >>> 
            (
                dir, inf_step, d_map_data, _max_yz_id, _max_z_id, 
                d_vertex_idx, d_inflate_result
            );
            cudaDeviceSynchronize();
            ros::Time time_2_cube_kernel = ros::Time::now();
            time_cuda_cube += (time_2_cube_kernel - time_2_cube_vertex_upload).toSec();

            // down the result about one direction's inflation
            ros::Time time_1_cube_vertex_download = ros::Time::now();
            cudaMemcpy(h_inflate_result, d_inflate_result, sizeof(bool), cudaMemcpyDeviceToHost);
            ros::Time time_2_cube_vertex_download = ros::Time::now();
            time_download_cube += (time_2_cube_vertex_download - time_1_cube_vertex_download).toSec();

            // judge whether we should take one step further in this direction
            if( *h_inflate_result == true ) // inflate one step success;
            {   
                switch(dir) 
                { 
                    case 0: //direction Y -
                        vertex_idx[8] -= inf_step;  vertex_idx[11] -= inf_step; vertex_idx[12] -= inf_step; vertex_idx[15] -= inf_step;
                        break;
                    case 1: //direction Y +
                        vertex_idx[9] += inf_step;  vertex_idx[10] += inf_step; vertex_idx[13] += inf_step; vertex_idx[14] += inf_step;
                        break; 
                    case 2: //direction X -
                        vertex_idx[2] -= inf_step;  vertex_idx[3]  -= inf_step; vertex_idx[6]  -= inf_step; vertex_idx[7]  -= inf_step;
                        break; 
                    case 3: //direction X +
                        vertex_idx[0] += inf_step;  vertex_idx[1]  += inf_step; vertex_idx[4]  += inf_step; vertex_idx[5]  += inf_step; 
                        break; 
                    case 4: //direction Z -
                        vertex_idx[20] -= inf_step; vertex_idx[21] -= inf_step; vertex_idx[22] -= inf_step; vertex_idx[23] -= inf_step;
                        break; 
                    case 5: //direction Z +  
                        vertex_idx[16] += inf_step; vertex_idx[17] += inf_step; vertex_idx[18] += inf_step; vertex_idx[19] += inf_step; 
                        break;  
                    default: 
                        break; 
                }
            }
        }

        bool is_inflate_conti = false;
        for(int vtx = 0; vtx < 8; vtx ++)
        {
            if( (vertex_idx_lst[vtx] != vertex_idx[vtx]) || (vertex_idx_lst[vtx + 8] != vertex_idx[vtx + 8]) || (vertex_idx_lst[vtx + 16] != vertex_idx[vtx + 16]) )
            {
                is_inflate_conti = true;
                break;
            }
        }

        if(is_inflate_conti == false)
            break;

        for(int vtx = 0; vtx < 8; vtx ++)
        { 
            vertex_idx_lst[vtx +  0] = vertex_idx[vtx +  0];
            vertex_idx_lst[vtx +  8] = vertex_idx[vtx +  8];
            vertex_idx_lst[vtx + 16] = vertex_idx[vtx + 16];
        }

        iter ++;
    }
}

void cudaPolytopeGeneration::polytopeCluster_cpu( int & cluster_grid_num, int & active_grid_num )
{   
    int candidate_grid_num;
    int can_x_idx, can_y_idx, can_z_idx;

    int itr_cluster_cnt = 0;
    while( itr_cluster_cnt < itr_cluster_max )
    {   
        candidate_grid_num = 0;
        for(int i = 0; i < active_grid_num; i++)
        {
            int cur_idx_x, cur_idx_y, cur_idx_z; 
            int nei_idx_x, nei_idx_y, nei_idx_z;

            cur_idx_x = active_xyz_id[3 * i];
            cur_idx_y = active_xyz_id[3 * i + 1];
            cur_idx_z = active_xyz_id[3 * i + 2];

            use_data[cur_idx_x * _max_yz_id + cur_idx_y * _max_z_id + cur_idx_z] = (uint8_t)1;

            // get all nearby voxels   
            for(int dx = -1; dx < 2; dx++)
            { 
                for(int dy = -1; dy < 2; dy++)
                {   
                    for(int dz = -1; dz < 2; dz++)
                    {   
                        if( dx == 0 && dy == 0 && dz == 0) continue;
                        //if( abs(dx) + abs(dy) + abs(dz) == 1 ) 
                        else
                        { 
                            nei_idx_x = cur_idx_x + dx;
                            nei_idx_y = cur_idx_y + dy;
                            nei_idx_z = cur_idx_z + dz;

                            if(nei_idx_x < 0 || nei_idx_x > _max_x_id - 1 
                            || nei_idx_y < 0 || nei_idx_y > _max_y_id - 1
                            || nei_idx_z < 0 || nei_idx_z > _max_z_id - 1 )
                                continue;

                            if(map_data    [nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 
                            || use_data    [nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 
                            || invalid_data[nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 
                            || inside_data [nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 )
                            {
                                continue;
                            }
                            else
                            {   
                                candidate_xyz_id[3 * candidate_grid_num    ] = nei_idx_x;
                                candidate_xyz_id[3 * candidate_grid_num + 1] = nei_idx_y;
                                candidate_xyz_id[3 * candidate_grid_num + 2] = nei_idx_z;

                                candidate_grid_num ++;
                                use_data[nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] = (uint8_t)1;
                            }
                        }
                    }
                }
            }
        }

        if( candidate_grid_num == 0 ) 
            break;

        active_grid_num = 0;
        for(int i = 0; i < candidate_grid_num; i++ )
        {
            // for each voxel in the candidate set, test if it preserves the convex hull property
            can_x_idx = candidate_xyz_id[3 * i];
            can_y_idx = candidate_xyz_id[3 * i + 1];
            can_z_idx = candidate_xyz_id[3 * i + 2];

            if( serialConvexTest(can_x_idx, can_y_idx, can_z_idx, cluster_grid_num, _max_yz_id, _max_z_id, cluster_xyz_id, inside_data, map_data ) )
            {   
                cluster_xyz_id[3 * cluster_grid_num]     = can_x_idx;
                cluster_xyz_id[3 * cluster_grid_num + 1] = can_y_idx;
                cluster_xyz_id[3 * cluster_grid_num + 2] = can_z_idx;

                active_xyz_id[3 * active_grid_num]       = can_x_idx;
                active_xyz_id[3 * active_grid_num + 1]   = can_y_idx;
                active_xyz_id[3 * active_grid_num + 2]   = can_z_idx;

                cluster_grid_num ++;
                active_grid_num ++;

                inside_data [can_x_idx * _max_yz_id + can_y_idx * _max_z_id + can_z_idx] = 0;
            }
            else
            {   
                invalid_data[can_x_idx * _max_yz_id + can_y_idx * _max_z_id + can_z_idx] = (uint8_t)1;
            }
        }
        
        if( active_grid_num == 0 ) 
            break;

        itr_cluster_cnt ++; 
    }
}

void cudaPolytopeGeneration::polytopeCluster_gpu(int & cluster_grid_num, int active_grid_num, 
                                                 double & time_upload, double & time_download, double & time_cuda)
{   
    int candidate_grid_num;
    ros::Time time_upload_11, time_upload_12, time_upload_21, time_upload_22;
    ros::Time time_download_1, time_download_2;
    ros::Time time_check_candidate_1, time_check_candidate_2;
    ros::Time time_start_gpu_cluster = ros::Time::now();

    double time_check_candidate = 0.0;

    int itr_cluster_cnt = 0;
    while( itr_cluster_cnt < itr_cluster_max )
    {   
        ros::Time time_ite = ros::Time::now();

        candidate_grid_num = 0;
        for(int i = 0; i < active_grid_num; i++)
        {
            int cur_idx_x, cur_idx_y, cur_idx_z; 
            int nei_idx_x, nei_idx_y, nei_idx_z;

            cur_idx_x = active_xyz_id[3 * i];
            cur_idx_y = active_xyz_id[3 * i + 1];
            cur_idx_z = active_xyz_id[3 * i + 2];

            use_data[cur_idx_x * _max_yz_id + cur_idx_y * _max_z_id + cur_idx_z] = (uint8_t)1;

            // get all nearby voxels   
            for(int dx = -1; dx < 2; dx++)
            { 
                for(int dy = -1; dy < 2; dy++)
                {   
                    for(int dz = -1; dz < 2; dz++)
                    {   
                        if( dx == 0 && dy == 0 && dz == 0) continue;
                        //if( abs(dx) + abs(dy) + abs(dz) == 1 ) 
                        else
                        { 
                            nei_idx_x = cur_idx_x + dx;
                            nei_idx_y = cur_idx_y + dy;
                            nei_idx_z = cur_idx_z + dz;

                            if(nei_idx_x < 0 || nei_idx_x > _max_x_id - 1 
                            || nei_idx_y < 0 || nei_idx_y > _max_y_id - 1
                            || nei_idx_z < 0 || nei_idx_z > _max_z_id - 1 )
                                continue;

                            if(map_data    [nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 
                            || use_data    [nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 
                            || invalid_data[nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 
                            || inside_data [nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] == 1 )
                            {
                                continue;
                            }
                            else
                            {   
                                candidate_xyz_id[3 * candidate_grid_num    ] = nei_idx_x;
                                candidate_xyz_id[3 * candidate_grid_num + 1] = nei_idx_y;
                                candidate_xyz_id[3 * candidate_grid_num + 2] = nei_idx_z;

                                candidate_grid_num ++;
                                use_data[nei_idx_x * _max_yz_id + nei_idx_y * _max_z_id + nei_idx_z] = (uint8_t)1;
                            }
                        }
                    }
                }
            }
        }
        
        // upload all candidate grids into the GPU
        time_upload_11 = ros::Time::now();
        cudaMemcpy(d_candidate_xyz_id, candidate_xyz_id, 3 * candidate_grid_num * sizeof(int),  cudaMemcpyHostToDevice);   

        time_upload_12 = ros::Time::now();
        time_upload += (time_upload_12 - time_upload_11).toSec();

#if DEBUG_VIS
        if(itr_cluster_cnt == 0)
            for(int i = 0; i < candidate_grid_num; i++)
            {
                vis_grid_id_x.push_back(candidate_xyz_id[3 * i + 0]);
                vis_grid_id_y.push_back(candidate_xyz_id[3 * i + 1]);
                vis_grid_id_z.push_back(candidate_xyz_id[3 * i + 2]);
            }
#endif

#if DEBUG_INFO_VERBOSE_LEVEL_1
        cout<<"convex cluster's total     grid num: "<<cluster_grid_num<<endl; 
        cout<<"last iteration's active    grid num: "<<active_grid_num<<endl;
        cout<<"this iteration's candidate grid num: "<<candidate_grid_num<<endl;
#endif

        if( candidate_grid_num == 0 ) break;

        ros::Time time_cuda_1 = ros::Time::now();

        int para_comp_num = candidate_grid_num * (cluster_grid_num + candidate_grid_num);
        threads_cvx.x  = min (1024, candidate_grid_num);
        blocks_cvx.x   = ceil(para_comp_num / threads_cvx.x) + 1;

        paraConvexTest <<< blocks_cvx, threads_cvx >>> 
        (
            d_map_data, d_inside_data, 
            d_candidate_xyz_id, d_cluster_xyz_id, d_result,
            _max_yz_id, _max_z_id,
            candidate_grid_num, cluster_grid_num
        );
        cudaDeviceSynchronize();
        
        threads_res_chk.x = min (1024, candidate_grid_num);
        blocks_res_chk.x  = ceil(candidate_grid_num / threads_res_chk.x) + 1;
        //cout<<"blocks_res_chk.x: "<<blocks_res_chk.x<<endl;

        paraResultCheck <<< blocks_res_chk, threads_res_chk >>> (
            d_result, d_can_can_result, d_can_clu_result,
            candidate_grid_num, cluster_grid_num );

        ros::Time time_cuda_2 = ros::Time::now();

// ###   debug data transfering blocking
        int can_can_total_num = candidate_grid_num * (candidate_grid_num - 1) / 2;
        time_download_1 = ros::Time::now();  

        cudaMemcpy(h_can_clu_result, d_can_clu_result, candidate_grid_num * sizeof(bool), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_can_can_result, d_can_can_result, can_can_total_num  * sizeof(bool), cudaMemcpyDeviceToHost);
        
        //cudaDeviceSynchronize();
        time_download_2 = ros::Time::now();
        time_download += (time_download_2 - time_download_1).toSec();

        time_check_candidate_1 = ros::Time::now();
        int can_x_idx, can_y_idx, can_z_idx;
        //memset(candidate_result, false, candidate_grid_num * sizeof(bool)); 
        std::fill(candidate_result, candidate_result + candidate_grid_num, false);
        
        active_grid_num = 0;
        int cluster_num_lst = cluster_grid_num;
        for(int i = 0; i < candidate_grid_num; i++)
        {   
            bool is_convex = true;
            if(h_can_clu_result[i] == false) 
            {
                is_convex = false;
            }
            else // check its relationship with its prior candidate grids
            {   
                int n = i + 1;
                int can_can_cnt_bias = n * (n - 1) / 2;
                
                for(int j = 0; j < i; j++)
                {
                    if( h_can_can_result[can_can_cnt_bias + j] == false && candidate_result[j] == true )
                    {
                        is_convex = false;
                        break;
                    }
                }
            }

            can_x_idx = candidate_xyz_id[3 * i];
            can_y_idx = candidate_xyz_id[3 * i + 1];
            can_z_idx = candidate_xyz_id[3 * i + 2];

            if(is_convex == true)
            {   
                candidate_result[i] = true;

                cluster_xyz_id[3 * cluster_grid_num]     = can_x_idx;
                cluster_xyz_id[3 * cluster_grid_num + 1] = can_y_idx;
                cluster_xyz_id[3 * cluster_grid_num + 2] = can_z_idx;

                active_xyz_id[3 * active_grid_num]       = can_x_idx;
                active_xyz_id[3 * active_grid_num + 1]   = can_y_idx;
                active_xyz_id[3 * active_grid_num + 2]   = can_z_idx;

                cluster_grid_num ++;
                active_grid_num ++;
            }
            else
            {   
                invalid_data[can_x_idx * _max_yz_id + can_y_idx * _max_z_id + can_z_idx] = (uint8_t)1;
            }
        }
        
        if( active_grid_num == 0 ) break;
        
        time_check_candidate_2 = ros::Time::now();
        time_check_candidate += (time_check_candidate_2 - time_check_candidate_1).toSec();

        time_upload_21 = ros::Time::now();
        //cudaMemcpyAsync(&d_cluster_xyz_id[3 * cluster_num_lst], &cluster_xyz_id[3 * cluster_num_lst], 3 * active_grid_num * sizeof(int), cudaMemcpyHostToDevice);   
        cudaMemcpy(&d_cluster_xyz_id[3 * cluster_num_lst], &cluster_xyz_id[3 * cluster_num_lst], 3 * active_grid_num * sizeof(int), cudaMemcpyHostToDevice);   
        time_upload_22 = ros::Time::now();

        time_upload += (time_upload_22 - time_upload_21).toSec();
        time_cuda   += (time_cuda_2 - time_cuda_1).toSec();
        ros::Time time_ite_2 = ros::Time::now();

#if DEBUG_INFO_VERBOSE_LEVEL_2
        ROS_WARN("time totally in one iteration: %f", (time_ite_2 - time_ite).toSec());
        ROS_WARN("time cost in cuda kernel: %f", (time_cuda_2 - time_cuda_1).toSec());
        ROS_WARN("time cost in check resuls: %f", (time_check_candidate_2 - time_check_candidate_1).toSec());
        ROS_WARN("time in data upload_step1: %f", (time_upload_12 - time_upload_11).toSec() );
        ROS_WARN("time in data upload_step2: %f", (time_upload_22 - time_upload_21).toSec() );
        ROS_WARN("time in data download : %f", (time_download_2 - time_download_1).toSec() );
        ROS_WARN("time so far: %f", (time_ite_2 - time_start_gpu_cluster).toSec());
#endif
        itr_cluster_cnt ++; 
    }

}

void cudaPolytopeGeneration::polygonGeneration( 
    vector<int> & cluster_x_idx, vector<int> & cluster_y_idx, vector<int> & cluster_z_idx )
{   
    //cout<<"call polygonGeneration"<<endl;
    flagClear();
    insideFlagUpload();
// ### 
    // Data uploading
    ros::Time time_start_gpu_computing = ros::Time::now();

    ros::Time time_1_cube = ros::Time::now();
    ros::Time time_2_cube = ros::Time::now();
    ros::Time time_1_poly = ros::Time::now();
    ros::Time time_2_poly = ros::Time::now();

    double time_upload_cube   = 0.0;
    double time_download_cube = 0.0;
    double time_cuda_cube = 0.0;

// ###
    // GPU-based cube inflation
    if(cluster_x_idx.size() == 1)
    {
        for(int vtx = 0; vtx < 8; vtx++)
        {   
            vertex_idx[vtx]      = cluster_x_idx[0];
            vertex_idx[vtx + 8 ] = cluster_y_idx[0];
            vertex_idx[vtx + 16] = cluster_z_idx[0];
        }        
    }
    else
    {
        int min_x, max_x, min_y, max_y, min_z, max_z;
        min_x = min_y = min_z = -100000;
        max_x = max_y = max_z = +100000;
        
        for(int i = 0; i < (int)cluster_x_idx.size(); i++)
        {
            min_x = cluster_x_idx[i] < min_x ? cluster_x_idx[i] : min_x;
            min_y = cluster_y_idx[i] < min_y ? cluster_y_idx[i] : min_y;
            min_z = cluster_z_idx[i] < min_z ? cluster_z_idx[i] : min_z;

            max_x = cluster_x_idx[i] > max_x ? cluster_x_idx[i] : max_x;
            max_y = cluster_y_idx[i] > max_y ? cluster_y_idx[i] : max_y;
            max_z = cluster_z_idx[i] > max_z ? cluster_z_idx[i] : max_z;
        }

        setVertexInitIndex(vertex_idx, min_x, min_y, min_z, max_x, max_y, max_z);
    }

    memcpy(vertex_idx_lst, vertex_idx, sizeof(int) * 24);

    time_1_cube = ros::Time::now();
#if _is_gpu_on_stage_1
    cubeInflation_gpu(vertex_idx_lst, vertex_idx, time_upload_cube, time_download_cube, time_cuda_cube);
#else
    cubeInflation_cpu(vertex_idx_lst, vertex_idx);
#endif

    time_2_cube = ros::Time::now();
    cluster_x_idx.clear(); cluster_y_idx.clear(); cluster_z_idx.clear();

    cube_grid_x.clear();
    cube_grid_y.clear();
    cube_grid_z.clear();
    getVoxelsInCube(vertex_idx, cube_grid_x, cube_grid_y, cube_grid_z, inside_data, _max_yz_id, _max_z_id);

    vector<int> cube_outside_grid_x, cube_outside_grid_y, cube_outside_grid_z;
    int tmp_x, tmp_y, tmp_z;

    if(cube_grid_x.size() == 1)
    {
        cube_outside_grid_x.push_back(cube_grid_x[0]);     
        cube_outside_grid_y.push_back(cube_grid_y[0]);     
        cube_outside_grid_z.push_back(cube_grid_z[0]);    

    }
    else
    {
        for(int i = 0; i < (int)cube_grid_x.size(); i++ )
        {   
            int idx = cube_grid_x[i] * _max_yz_id + cube_grid_y[i] * _max_z_id + cube_grid_z[i];
            
            use_data[idx] = (uint8_t)1;

            int is_inside = 1;       
            for(int dx = -1; dx < 2; dx++)
            {   
                for(int dy = -1; dy < 2; dy++)
                {   
                    for(int dz = -1; dz < 2; dz++)
                    {   
                        if(dx == 0 && dy == 0 && dz == 0)
                            continue;

                        tmp_x = cube_grid_x[i] + dx;
                        tmp_y = cube_grid_y[i] + dy;
                        tmp_z = cube_grid_z[i] + dz;

                        if( tmp_x >= 0 && tmp_x < _max_x_id && tmp_y >= 0 && tmp_y < _max_y_id && tmp_z >= 0 && tmp_z < _max_z_id )
                        {
                            int idx_tmp = tmp_x * _max_yz_id + tmp_y * _max_z_id + tmp_z;
                            is_inside *= inside_data[idx_tmp];
                        }
                        else
                            is_inside = 0;   
                    }
                }
            }

            if( is_inside < 1 ) // this grid is actually outside the cube
            {
                cube_outside_grid_x.push_back(cube_grid_x[i]);     
                cube_outside_grid_y.push_back(cube_grid_y[i]);     
                cube_outside_grid_z.push_back(cube_grid_z[i]);     
            }
        }
    }
    
    for(int i = 0; i < (int)cube_outside_grid_x.size(); i++)
    {
        inside_data[cube_outside_grid_x[i] * _max_yz_id + cube_outside_grid_y[i] * _max_z_id + cube_outside_grid_z[i]] = (uint8_t)0;
    }

    insideFlagUpload();      

// ####
    // GPU-based convex clustering
    int init_cluster_grid_num = cube_outside_grid_x.size();
    int active_grid_num = init_cluster_grid_num;

    //ROS_WARN("active_grid_num, after cube inflation, is : %d", active_grid_num);

    for(int i = 0; i < init_cluster_grid_num; i++)
    {
        cluster_xyz_id[3 * i]     = cube_outside_grid_x[i];
        cluster_xyz_id[3 * i + 1] = cube_outside_grid_y[i];
        cluster_xyz_id[3 * i + 2] = cube_outside_grid_z[i];

        active_xyz_id[3 * i]      = cube_outside_grid_x[i];
        active_xyz_id[3 * i + 1]  = cube_outside_grid_y[i];
        active_xyz_id[3 * i + 2]  = cube_outside_grid_z[i];
    }

    // degenerate case,
    if( abs(vertex_idx[0 + 7]  - vertex_idx[0 + 1] ) == 0 || abs(vertex_idx[8 + 7]  - vertex_idx[8 + 1] ) == 0 || abs(vertex_idx[16 + 7] - vertex_idx[16 + 1]) == 0 )
    {   
        for(int i = 0; i < init_cluster_grid_num; i++)
        {
            cluster_x_idx.push_back(cluster_xyz_id[3 * i]);
            cluster_y_idx.push_back(cluster_xyz_id[3 * i + 1]);
            cluster_z_idx.push_back(cluster_xyz_id[3 * i + 2]);
        }
        return;
    }

    time_1_poly = ros::Time::now();
    double time_cuda_poly     = 0.0;
    double time_upload_poly   = 0.0;
    double time_download_poly = 0.0;

    ros::Time time_upload_init1 = ros::Time::now();
    cudaMemcpy(d_cluster_xyz_id, cluster_xyz_id, 3 * init_cluster_grid_num * sizeof(int), cudaMemcpyHostToDevice);   
    ros::Time time_upload_init2 = ros::Time::now();
    time_upload_poly += (time_upload_init2 - time_upload_init1).toSec();

    int cluster_grid_num = init_cluster_grid_num;

if  (_is_gpu_on_stage_2)
    polytopeCluster_gpu(cluster_grid_num, active_grid_num, time_upload_poly, time_download_poly, time_cuda_poly);
else 
    polytopeCluster_cpu( cluster_grid_num, active_grid_num );
    time_2_poly = ros::Time::now();

if ( DEBUG_INFO_VERBOSE_LEVEL_1 && _is_gpu_on_stage_1 )
{
    ROS_INFO("<Inflation> Time in cuda kernel: %f", time_cuda_cube );
    ROS_INFO("<Inflation> Time in data upload: %f", time_upload_cube );
    ROS_INFO("<Inflation> Time in data download: %f", time_download_cube );
}

if ( DEBUG_INFO_VERBOSE_LEVEL_1 && _is_gpu_on_stage_2 )
{
    ROS_INFO("<Clustering> Time in cuda kernel: %f",   time_cuda_poly     );
    ROS_INFO("<Clustering> Time in data upload: %f",   time_upload_poly   );
    ROS_INFO("<Clustering> Time in data download: %f", time_download_poly );
}

    ros::Time time_finish_gpu_computing = ros::Time::now();

    for(int i = 0; i < cluster_grid_num; i++)
    {
        cluster_x_idx.push_back(cluster_xyz_id[3 * i]);
        cluster_y_idx.push_back(cluster_xyz_id[3 * i + 1]);
        cluster_z_idx.push_back(cluster_xyz_id[3 * i + 2]);
    }

    cout<<"[polyhedron_generator]{GPU} finish gpu cluster"<<endl;
}   