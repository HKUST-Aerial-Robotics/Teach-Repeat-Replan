#include <iostream>
#include <limits>
#include <math.h>
using namespace std;

float signum_cpu(const int & x);
float mod_cpu(const float & value, const float & modulus);
float intbound_cpu(float s, int ds);

bool serialConvexTest( const int & can_x_index, const int & can_y_index, const int & can_z_index, const int & cluster_grid_num,
                       const int & max_yz_id, const int & max_z_id,
                       const int * cluster_xyz_id, const uint8_t * inside_data, const uint8_t * map_data);