#include "polyhedron_generator/cluster_engine_cpu.h"

inline float signum_cpu(const int & x)
{
    return x == 0 ? 0 : x < 0 ? -1 : 1;
}

inline float mod_cpu(const float & value, const float & modulus)
{
  return fmod(fmod(value, modulus) + modulus,  modulus);
}

inline float intbound_cpu(float s, int ds)
{
  if (ds == 0)
  {
    return std::numeric_limits<double>::max();
  }
  else if (ds < 0)
  {
    return intbound_cpu(-s, -ds);
  }
  else
  {
    s = mod_cpu(s, 1.0f);

    return (1-s)/ds;
  }
}

bool serialConvexTest( const int & can_x_index, const int & can_y_index, const int & can_z_index, const int & cluster_grid_num,
                       const int & max_yz_id, const int & max_z_id,
                       const int * cluster_xyz_id, const uint8_t * inside_data, const uint8_t * map_data)
{   
    int index_1_med[3];
    int index_med[3];
    index_1_med[0] = can_x_index / 2;
    index_1_med[1] = can_y_index / 2;
    index_1_med[2] = can_z_index / 2;

    int index_2  [3];
    for(int i = cluster_grid_num - 1; i >= 0 ; i--)
    //for(int i = 0; i < cluster_grid_num ; i++)
    {   
        index_2[0] = cluster_xyz_id[3 * i];
        index_2[1] = cluster_xyz_id[3 * i + 1];
        index_2[2] = cluster_xyz_id[3 * i + 2];
        
        if( inside_data[index_2[0] * max_yz_id + index_2[1] * max_z_id + index_2[2]] == 0 ) 
        {   
            index_med[0] = index_1_med[0] + (index_2[0] >> 1);
            index_med[1] = index_1_med[1] + (index_2[1] >> 1);
            index_med[2] = index_1_med[2] + (index_2[2] >> 1);

            if( inside_data[index_med[0] * max_yz_id + index_med[1] * max_z_id + index_med[2]] == 1 )
                continue;

            int x,y,z, endX, endY, endZ;
            int dx,dy,dz, stepX, stepY, stepZ;
            float tMaxX, tMaxY, tMaxZ;
            float tDeltaX, tDeltaY, tDeltaZ;

            //cout<<"test an outer grid"<<endl;
            x = can_x_index;
            y = can_y_index;
            z = can_z_index;
            
            endX = index_2[0];
            endY = index_2[1];
            endZ = index_2[2];

            dx = endX - x;
            dy = endY - y;
            dz = endZ - z;

            stepX = signum_cpu(dx);
            stepY = signum_cpu(dy);
            stepZ = signum_cpu(dz);

            //if (stepX == 0 && stepY == 0 && stepZ == 0) { continue;}

            tMaxX = intbound_cpu(0.5, dx);
            tMaxY = intbound_cpu(0.5, dy);
            tMaxZ = intbound_cpu(0.5, dz);

            // The change in t when taking a step (always positive).
            tDeltaX = ((float)stepX) / dx;
            tDeltaY = ((float)stepY) / dy;
            tDeltaZ = ((float)stepZ) / dz;

            while (true)
            {   
                if (tMaxX < tMaxY)
                {
                    if (tMaxX < tMaxZ)
                    {
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

                if(x == endX && y == endY && z == endZ)
                    break;

                if( inside_data[x * max_yz_id + y * max_z_id + z] == 1 ) // has traced an inner grid of this cluster, no need to continue tracing 
                {   
                    break;
                }
                else if( map_data[x * max_yz_id + y * max_z_id + z] == 1 ) // here we find a obstacle on the ray
                {   
                    return false;
                }
            }

        }
    }

    return true;
}