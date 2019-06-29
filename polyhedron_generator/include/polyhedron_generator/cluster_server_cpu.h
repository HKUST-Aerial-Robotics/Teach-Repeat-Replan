#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <stdint.h>

class cudaPolytopeGeneration
{
	private:
		int _cluster_buffer_size, _candidate_buffer_size;
		int _cluster_buffer_size_square;

    	int _max_x_id, _max_y_id, _max_z_id, _grid_num;
    	int _max_yz_id;

    	int itr_inflate_max, itr_cluster_max;

    	int inf_step;
	    double _resolution;

    	uint8_t * map_data, * use_data, * inside_data, * invalid_data;

    	int * cluster_xyz_id, * candidate_xyz_id, * active_xyz_id;
	    bool * candidate_result, * h_inflate_result;
	    bool * h_can_can_result, * h_can_clu_result;
	    int * vertex_idx, * vertex_idx_lst;

	public:
		cudaPolytopeGeneration(){};
		
		~cudaPolytopeGeneration()
		{
		    delete [] map_data;
		    delete [] use_data;
		    delete [] inside_data;
		    delete [] invalid_data;
		   
		    delete [] candidate_result;
		    delete [] active_xyz_id;
		    delete [] vertex_idx;
		    delete [] vertex_idx_lst;

		    delete [] cluster_xyz_id;
		    delete [] candidate_xyz_id;
		    delete [] h_can_can_result;
		    delete [] h_can_clu_result;
		    delete [] h_inflate_result;

		};

		void paramSet( bool is_gpu_on_stage_1, bool is_gpu_on_stage_2, bool is_cluster_on,
    		 const int & max_x_id,  const int & max_y_id,  const int & max_z_id, double resolution, double itr_inflate_max_, double itr_cluster_max_ );

		void setVertexInitIndex(int * vertex_idx, int min_x, int min_y, int min_z, int max_x, int max_y, int max_z);
		
		void getVoxelsInCube(int * vertex_idx, std::vector<int> & cube_grid_x, std::vector<int> & cube_grid_y, std::vector<int> & cube_grid_z, 
			uint8_t * inside_data, int map_yz_size, int map_z_size );

		void inflateX_n(int * vertex_idx);
		void inflateX_p(int * vertex_idx);
		void inflateY_n(int * vertex_idx);
		void inflateY_p(int * vertex_idx);
		void inflateZ_n(int * vertex_idx);
		void inflateZ_p(int * vertex_idx);

		void cubeInflation_cpu( int * vertex_idx_lst, int * vertex_idx );
		void polytopeCluster_cpu( int & cluster_grid_num, int & active_grid_num );

		void polygonGeneration( std::vector<int> & cluster_x_idx, std::vector<int> & cluster_y_idx, std::vector<int> & cluster_z_idx);
		
		void mapClear();
		void mapUpload();
		void flagClear();
		
		void setObs(const int & idx);
		void setObs(const int & id_x, const int & id_y, const int & id_z);
		void setFr (const int & idx);
		void setFr (const int & id_x, const int & id_y, const int & id_z);

		uint8_t IndexQuery(const int & id_x, const int & id_y, const int & id_z)
		{
			return map_data[id_x * _max_yz_id + id_y * _max_z_id + id_z];
		};		
};