#include <global_planner/utils/poly_utils.h>

void polyhedronGenerator::coord2Index(int & id_x, int & id_y, int & id_z, const double & pt_x, const double & pt_y, const double & pt_z)
{   
    id_x = min( max( int( (pt_x - _map_lower(0) ) * _inv_resolution), 0), _max_x_id - 1),
    id_y = min( max( int( (pt_y - _map_lower(1) ) * _inv_resolution), 0), _max_y_id - 1),
    id_z = min( max( int( (pt_z - _map_lower(2) ) * _inv_resolution), 0), _max_z_id - 1);                  
}

Vector3i polyhedronGenerator::coord2Index(Vector3d coord)
{
    int id_x = min( max( int( (coord(0) - _map_lower(0) ) * _inv_resolution), 0), _max_x_id - 1);
    int id_y = min( max( int( (coord(1) - _map_lower(1) ) * _inv_resolution), 0), _max_y_id - 1);
    int id_z = min( max( int( (coord(2) - _map_lower(2) ) * _inv_resolution), 0), _max_z_id - 1);                  

    Vector3i index(id_x, id_y, id_z);
    return index;
}

Vector3d polyhedronGenerator::index2Coord(Vector3i index)
{
    Vector3d pt;

    pt(0) = index(0) * _resolution + 0.5 * _resolution + _map_lower(0);
    pt(1) = index(1) * _resolution + 0.5 * _resolution + _map_lower(1);
    pt(2) = index(2) * _resolution + 0.5 * _resolution + _map_lower(2);

    return pt;
}

Vector3d polyhedronGenerator::index2Coord(int id_x, int id_y, int id_z)
{
    Vector3d pt;

    pt(0) = id_x * _resolution + 0.5 * _resolution + _map_lower(0);
    pt(1) = id_y * _resolution + 0.5 * _resolution + _map_lower(1);
    pt(2) = id_z * _resolution + 0.5 * _resolution + _map_lower(2);

    return pt;
}

bool polyhedronGenerator::isOutsidePolytope( const Vector3d & cur_coord, const decomp_cvx_space::Polytope & polytope)
{
    for( auto plane: polytope.planes )
    {   
        double res = cur_coord.dot(plane.head(3)) + plane(3);
        if( res > 0.01 )
            return true;
    }

    return false;
}

bool polyhedronGenerator::isInsidePolytope( const Vector3d & cur_coord, const decomp_cvx_space::Polytope & polytope)
{   
    return !isOutsidePolytope(cur_coord, polytope);
}

bool polyhedronGenerator::isOutsideLatestPolytope( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor)
{   
    return isOutsidePolytope( cur_coord, corridor.polyhedrons.back() );
}

bool polyhedronGenerator::isInsideLastSecondPolytope( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor)
{   
    if( corridor.polyhedrons.size() > 1 )
        return isInsidePolytope( cur_coord, corridor.polyhedrons[corridor.polyhedrons.size() - 2] );
    else
        return false;
}

bool polyhedronGenerator::isOutsideCorridor( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor)
{   
    for(auto ptly: corridor.polyhedrons)
    {
        if( isOutsidePolytope( cur_coord, ptly ) == false ) // this point inside a polytope
            return false; // it's inside the corridor
    }

    return true; // now inside any polytopes, it's outside the corridor
}

bool polyhedronGenerator::isInsideLatestPolytope( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor)
{   
    return !( isOutsidePolytope( cur_coord, corridor.polyhedrons.back() ) );
}

bool polyhedronGenerator::isInsideCorridor( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor)
{   
    return !isOutsideCorridor(cur_coord, corridor);
}

double _max_res      = 999999.0;
double _zero_epislon = 0.0001;
MatrixXd polyhedronGenerator::getVerticesPlane( const pair<MatrixXd, VectorXd> hrep, const MatrixXd & vertices3D)
{   
    MatrixXd hrep_a = hrep.first; 
    VectorXd hrep_b = hrep.second;

    int vertices_num = hrep_a.rows();
    int vertices_set_num = vertices3D.rows();
    
    MatrixXd vertices(vertices_num, 3); 
    
    for(int i = 0; i< vertices_num; i++)
    {   
        double min_res = _max_res;
        double res;
        int    min_index = 0;

        for(int k = 0; k < vertices_set_num; k++)
        {   
            res = hrep_a(i, 0) * vertices3D(k, 0) + hrep_a(i, 1) * vertices3D(k, 1) + hrep_a(i, 2) * vertices3D(k, 2) - hrep_b(i);
            if(fabs(res) < min_res)
            {   
                min_res   = fabs(res);
                min_index = k;
            }
        }

        vertices.row(i) = vertices3D.row(min_index);
    }

    return vertices;
}

pair<decomp_ros_msgs::Polyhedron, decomp_cvx_space::Polytope> polyhedronGenerator::polyHrep2Utils(const pair<MatrixXd, VectorXd> & hrep, const MatrixXd & cur_vertices, bool is_degenarate)
{   
    MatrixXd vertices = getVerticesPlane(hrep, cur_vertices);

    decomp_ros_msgs::Polyhedron msg;
    decomp_cvx_space::Polytope pltp;// = new decomp_cvx_space::Polytope();

    Vector3d center = VectorXd::Zero(3);

    for(int i = 0; i < vertices.rows(); i++)
    {
        center(0) = center(0) + vertices(i, 0);
        center(1) = center(1) + vertices(i, 1);
        center(2) = center(2) + vertices(i, 2);
    }

    center = center / vertices.rows();
    pltp.setCenter(center);

    //    hrep: ax + by + cz = K;
    //       => ax + by + cz + K = 0;
    MatrixXd normals =   hrep.first;
    VectorXd Ks      = - hrep.second;

    for(int i = 0; i < normals.rows(); i++)
    {   
        Vector3d normal = normals. row(i);
        Vector3d vertex = vertices.row(i);

        double L = normal.norm();
        double K = Ks(i)  / L;
        normal   = normal / L;

        if( (center - vertex).dot(normal) >= 0)
        {
            normal = -1.0 * normal;
            K = -K;
        }

        Vector4d plane(normal(0), normal(1), normal(2), K);
        geometry_msgs::Point pt, n;
        pt.x = vertex(0);
        pt.y = vertex(1);
        pt.z = vertex(2);

        if( !is_degenarate )
        {
            if( fabs(normal(0)) > _zero_epislon && fabs(normal(1)) < _zero_epislon && fabs(normal(2)) < _zero_epislon )
            {
                pt.x += _resolution * 0.5 / normal(0);
                plane(3) -= _resolution * 0.5;
            }
            else if( fabs(normal(1)) > _zero_epislon && fabs(normal(0)) < _zero_epislon && fabs(normal(2)) < _zero_epislon )
            {
                pt.y += _resolution * 0.5 / normal(1);
                plane(3) -= _resolution * 0.5;
            }
            else if( fabs(normal(2)) > _zero_epislon && fabs(normal(0)) < _zero_epislon && fabs(normal(1)) < _zero_epislon )
            {
                pt.z += _resolution * 0.5 / normal(2);
                plane(3) -= _resolution * 0.5;
            }   
        }

        n.x = normal(0);
        n.y = normal(1);
        n.z = normal(2);

        pltp.appendPlane(plane);
        msg.points.push_back(pt);
        msg.normals.push_back(n);
    }
    
    for(int i = 0; i < cur_vertices.rows(); i++)
    {
        pltp.appendVertex(cur_vertices.row(i));
    }

    return make_pair(msg, pltp);
}

void polyhedronGenerator::getVoxelVertex( vector<Vector3d> & coordCorners, const vector<Vector3d> & coords)
{   
    Vector3d coordCorner;
    for(auto coordCenter: coords)
    {
        for(int x = -1; x < 2; x+=2){
            for(int y = -1; y < 2; y+=2){
                for(int z = -1; z < 2; z+=2){
                    Vector3d dir(x, y, z);
                    coordCorner = coordCenter + dir * _resolution * 0.5;
                    coordCorners.push_back(coordCorner);
                }
            }
        }
    }
}

Vector3d polyhedronGenerator::round2Voxel(const Vector3d & coord)
{
    Vector3d voxel_coord;
    
    voxel_coord(0) = round( (coord(0) - _map_lower(0)) * _inv_resolution * 2.0 ) * _resolution * 0.5 + _map_lower(0);
    voxel_coord(1) = round( (coord(1) - _map_lower(1)) * _inv_resolution * 2.0 ) * _resolution * 0.5 + _map_lower(1);
    voxel_coord(2) = round( (coord(2) - _map_lower(2)) * _inv_resolution * 2.0 ) * _resolution * 0.5 + _map_lower(2);

    return voxel_coord;
}

bool polyhedronGenerator::checkDegeneratePoly(const vector<int> & cluster_x_idx, const vector<int> & cluster_y_idx, const vector<int> & cluster_z_idx)
{   
    int grid_num = cluster_x_idx.size();

    bool is_deg_x = true;
    bool is_deg_y = true;
    bool is_deg_z = true;

    for(int i = 0; i < grid_num - 1; i++)
    {
        if( cluster_x_idx[i] != cluster_x_idx[i+1] )
        {   
            is_deg_x = false;
            break;
        }
    }

    for(int i = 0; i < grid_num - 1; i++)
    {
        if( cluster_y_idx[i] != cluster_y_idx[i+1] )
        {   
            is_deg_y = false;
            break;
        }
        
    }

    for(int i = 0; i < grid_num - 1; i++)
    {
        if( cluster_z_idx[i] != cluster_z_idx[i+1] )
        {   
            is_deg_z = false;
            break;
        }
    }

    return (is_deg_x || is_deg_y || is_deg_z);
}

pair <MatrixXd, bool> polyhedronGenerator::getConvexPoly( Vector3d seed_pt )
{
    vector<Vector3d> seed_pts = {seed_pt};

    return getConvexPoly(seed_pts);
}

pair <MatrixXd, bool> polyhedronGenerator::getConvexPoly( vector<Vector3d> seed_pts )
{
    vector<int> cluster_x_idx, cluster_y_idx, cluster_z_idx;
    
#define cluster_grid_buffer 400000
    cluster_x_idx.reserve(cluster_grid_buffer);
    cluster_y_idx.reserve(cluster_grid_buffer);
    cluster_z_idx.reserve(cluster_grid_buffer);

    for(int i = 0; i < (int)seed_pts.size(); i++)
    {
        Vector3i seed_idx = coord2Index( seed_pts[i] );
        cluster_x_idx.push_back(seed_idx(0));
        cluster_y_idx.push_back(seed_idx(1));
        cluster_z_idx.push_back(seed_idx(2));
    }

    _cuda_polytope_generator->polygonGeneration( cluster_x_idx, cluster_y_idx, cluster_z_idx );

    QuickHull<double> quickHull; 
    vector<Vector3<double>> grids_set;
    MatrixXd vertices;

#define use_voxel_cornor 0
    bool is_degenarate = false;
    if(checkDegeneratePoly(cluster_x_idx, cluster_y_idx, cluster_z_idx))
    {   
        //ROS_WARN("Degenerate to one plane case");
        is_degenarate = true;
        vector<Vector3d> coordCorners;
        for(int i = 0; i < (int)cluster_x_idx.size(); i++ )
        {   
            Vector3d coordCenter = index2Coord(cluster_x_idx[i], cluster_y_idx[i], cluster_z_idx[i]);
            coordCorners.clear();
            vector<Vector3d> coords = {coordCenter};
            getVoxelVertex(coordCorners, coords);
            
            for(auto coord: coordCorners)
            {
                Vector3<double> point(coord(0), coord(1), coord(2));
                grids_set.push_back(point);
            }
        }
        
        auto hull = quickHull.getConvexHull(grids_set, true, false); 
        auto vertexBuffer = hull.getVertexBuffer();

        int vertex_num = (int)vertexBuffer.size();
        vertices.resize(vertex_num, 3);

        for(int i = 0; i < vertex_num; i++)
        {
            Vector3d vertex(vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z);        
            vertex = round2Voxel(vertex);
            vertices.row(i) = vertex;
        }
    }
    else
    {
        
#if use_voxel_cornor 

        vector<Vector3d> coordCorners;
        for(int i = 0; i < (int)cluster_x_idx.size(); i++ )
        {   
            Vector3d coordCenter = index2Coord(cluster_x_idx[i], cluster_y_idx[i], cluster_z_idx[i]);

            coordCorners.clear();
            vector<Vector3d> coords = {coordCenter};
            getVoxelVertex(coordCorners, coords);
            
            for(auto coord: coordCorners)
            {   
                Vector3<double> point(coord(0), coord(1), coord(2));
                grids_set.push_back(point);
            }            
        }
#else  
        for(int i = 0; i < (int)cluster_x_idx.size(); i++ )
        {   
            Vector3d coordCenter = index2Coord(cluster_x_idx[i], cluster_y_idx[i], cluster_z_idx[i]);
            Vector3<double> point(coordCenter(0), coordCenter(1), coordCenter(2));
            
            grids_set.push_back(point);
        }
#endif           
        auto hull = quickHull.getConvexHull(grids_set, true, false); 
        auto vertexBuffer = hull.getVertexBuffer();

        int vertex_num = (int)vertexBuffer.size();
        vertices.resize(vertex_num, 3);

        for(int i = 0; i < vertex_num; i++)
        {
            Vector3d vertex(vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z);        

#if use_voxel_cornor            
            vertex = round2Voxel(vertex);
#else
            vertex = index2Coord(coord2Index(vertex));
#endif

            vertices.row(i) = vertex;
        }
    }

    return make_pair(vertices, is_degenarate);
}

int polyhedronGenerator::corridorInsertGeneration( const vector<Vector3d> & coordSet, decomp_ros_msgs::PolyhedronArray & poly_array_msg)
{   
    // insert the generated polyhedrons into the beginning of the existing corridor
    if( !has_map ) return 0;

    Vector3d cur_coord(-inf, -inf, -inf);
    Vector3d lst_coord(-inf, -inf, -inf);
    decomp_cvx_space::FlightCorridor beg_corridor;
    decomp_cvx_space::FlightCorridor tmp_corridor;

    Polyhedron poly_cdd;
    for ( int pts_cnt = 0; pts_cnt < (int)coordSet.size(); pts_cnt ++ )
    {
        cur_coord = coordSet[pts_cnt];
        cur_coord = index2Coord( coord2Index(cur_coord) );
        
        if( cur_coord == lst_coord )
            continue;

        // specially deal with the case: the current position of the drone is already in the first polytope of the flight corridor
        if( isInsidePolytope(cur_coord, corridor.polyhedrons.front()) )
        {
            // in this case, no need to continue the convexify procedure
            decomp_cvx_space::Polytope polytope = corridor.polyhedrons.front();
            polytope.setSeed(cur_coord);
            beg_corridor.polyhedrons.push_back(polytope);
            
            break;
        }

        if( beg_corridor.isEmpty() || isOutsideLatestPolytope(cur_coord, beg_corridor) )
        {   
            // #### : Find the convex latgest free space     
            auto poly_res = getConvexPoly(cur_coord);
            MatrixXd vertices = poly_res.first;
            bool is_degen = poly_res.second;
            if( poly_cdd.setVertices(vertices) && poly_cdd.lastErrorMessage() == "*No Error found.\n" )
            {
                auto poly_pair = polyHrep2Utils(poly_cdd.hrep(), vertices, is_degen);
                
                decomp_ros_msgs::Polyhedron poly_msg = poly_pair.first;
                poly_array_msg.polyhedrons.push_back(poly_msg);
                poly_array_msg.ids.push_back(poly_id); 
                poly_id ++;

                decomp_cvx_space::Polytope polytope = poly_pair.second;
                polytope.setSeed(cur_coord);

                beg_corridor.polyhedrons.push_back(polytope);
            }
            else{
                return 0;
            }
        }
        
        lst_coord = cur_coord;
    }

    for(auto pltp: beg_corridor.polyhedrons)
        tmp_corridor.polyhedrons.push_back(pltp);

    for(auto pltp: corridor.polyhedrons)
        tmp_corridor.polyhedrons.push_back(pltp);

    corridor = tmp_corridor;
    return 1;
}


int polyhedronGenerator::corridorIncreGeneration( const vector<Vector3d> & coordSet, decomp_ros_msgs::PolyhedronArray & poly_array_msg)
{   
    if( !has_map ) return 0;

    int return_flag = 0;
    Vector3d cur_coord(-inf, -inf, -inf);
    Vector3d lst_coord( inf,  inf,  inf);
    
    Polyhedron poly_cdd;
    for ( int pts_cnt = 0; pts_cnt < (int)coordSet.size(); pts_cnt ++ )
    {
        cur_coord = coordSet[pts_cnt];
        cur_coord = index2Coord( coord2Index(cur_coord) );
        
        if( cur_coord == lst_coord )
            continue;

        if( isInsideLastSecondPolytope(cur_coord, corridor) ){
            return_flag = 1;
            corridor.polyhedrons.pop_back();
            poly_array_msg.polyhedrons.pop_back();
            poly_array_msg.ids.pop_back(); 
            poly_id --;
        }
        
        if( corridor.isEmpty() || isOutsideLatestPolytope(cur_coord, corridor) ){   
            return_flag = 1;
            
            // #### : Find the convex latgest free space     
            auto poly_res = getConvexPoly(cur_coord);
            MatrixXd vertices = poly_res.first;
            bool is_degen = poly_res.second;
            if( poly_cdd.setVertices(vertices) && poly_cdd.lastErrorMessage() == "*No Error found.\n" ){
                auto poly_pair = polyHrep2Utils(poly_cdd.hrep(), vertices, is_degen);
                
                decomp_ros_msgs::Polyhedron poly_msg = poly_pair.first;
                poly_array_msg.polyhedrons.push_back(poly_msg);
                poly_array_msg.ids.push_back(poly_id); 
                poly_id ++;

                decomp_cvx_space::Polytope polytope = poly_pair.second;
                polytope.setSeed(cur_coord);

                corridor.polyhedrons.push_back(polytope);
            }
            else{
                return 0;
            }
        }
        
        lst_coord = cur_coord;
    }

    return return_flag;
}

void polyhedronGenerator::corridorGeneration(const vector<Vector3d> & gridPath, decomp_ros_msgs::PolyhedronArray & poly_array_msg)
{   
    if( !has_map ) return;

    Vector3d cur_coord(-inf, -inf, -inf);
    Vector3d lst_coord(-inf, -inf, -inf);
    
    Polyhedron poly_cdd;
    for ( int pts_cnt = 0; pts_cnt < (int)gridPath.size(); pts_cnt ++ )
    {
        cur_coord = gridPath[pts_cnt];
        cur_coord = index2Coord( coord2Index(cur_coord) );
        
        if( cur_coord == lst_coord )
            continue;

        if( isInsideLastSecondPolytope(cur_coord, corridor) )
        {
            corridor.polyhedrons.pop_back();
            poly_array_msg.polyhedrons.pop_back();
        }
        
        if( corridor.isEmpty() || isOutsideLatestPolytope(cur_coord, corridor) )
        {  
            // #### : Find the convex latgest free space     
            auto poly_res = getConvexPoly(cur_coord);
            MatrixXd vertices = poly_res.first;
            bool is_degen = poly_res.second;
            if( poly_cdd.setVertices(vertices) && poly_cdd.lastErrorMessage() == "*No Error found.\n" )
            {
                auto poly_pair = polyHrep2Utils(poly_cdd.hrep(), vertices, is_degen);
                
                decomp_ros_msgs::Polyhedron poly_msg = poly_pair.first;
                poly_array_msg.polyhedrons.push_back(poly_msg);

                decomp_cvx_space::Polytope polytope = poly_pair.second;
                polytope.setSeed(cur_coord);
                //polytope.printPlanes();

                corridor.polyhedrons.push_back(polytope);
            }
            else
            {
                ROS_ERROR("[T-R Node] cdd-lib ERROR, corridor generation broke");
                return;
            }
        }
        
        lst_coord = cur_coord;
    }
}