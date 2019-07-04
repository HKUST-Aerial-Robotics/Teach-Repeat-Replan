#include "polyhedron_generator/cluster_server_cpu.h"
#include "polyhedron_generator/cluster_engine_cpu.h"

using namespace std;

#define DEBUG_VIS 0
#define DEBUG_INFO_VERBOSE_LEVEL_1 0
#define DEBUG_INFO_VERBOSE_LEVEL_2 0

void cudaPolytopeGeneration::mapClear()
{   
    memset(map_data,     (uint8_t)0, _grid_num*sizeof(uint8_t)); 
}

void cudaPolytopeGeneration::mapUpload()
{       
    return;
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

    _cluster_buffer_size   = 50000;
    _candidate_buffer_size = 10000;
    _cluster_buffer_size_square = _candidate_buffer_size * _candidate_buffer_size;

    map_data     = new uint8_t[_grid_num];
    use_data     = new uint8_t[_grid_num];
    invalid_data = new uint8_t[_grid_num];
    inside_data  = new uint8_t[_grid_num];

    candidate_result  = new bool[_candidate_buffer_size];
    active_xyz_id     = new int[_candidate_buffer_size * 3];
    vertex_idx        = new int[24];
    vertex_idx_lst    = new int[24];

    h_inflate_result = new bool;
    cluster_xyz_id   = new int[_cluster_buffer_size   * 3];
    candidate_xyz_id = new int[_candidate_buffer_size * 3];
    h_can_can_result = new bool[_cluster_buffer_size_square];
    h_can_clu_result = new bool[_candidate_buffer_size];

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

void cudaPolytopeGeneration::polygonGeneration( 
    vector<int> & cluster_x_idx, vector<int> & cluster_y_idx, vector<int> & cluster_z_idx )
{   
    flagClear();
    
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

    cubeInflation_cpu(vertex_idx_lst, vertex_idx);

    cluster_x_idx.clear(); cluster_y_idx.clear(); cluster_z_idx.clear();

    std::vector<int> cube_grid_x, cube_grid_y, cube_grid_z;
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

    int init_cluster_grid_num = cube_outside_grid_x.size();
    int active_grid_num = init_cluster_grid_num;

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

    int cluster_grid_num = init_cluster_grid_num;
    polytopeCluster_cpu( cluster_grid_num, active_grid_num );

    for(int i = 0; i < cluster_grid_num; i++)
    {
        cluster_x_idx.push_back(cluster_xyz_id[3 * i]);
        cluster_y_idx.push_back(cluster_xyz_id[3 * i + 1]);
        cluster_z_idx.push_back(cluster_xyz_id[3 * i + 2]);
    }

    cout<<"[polyhedron_generator]{CPU} finish cpu cluster"<<endl;
}   