#include <global_planner/utils/a_star.h>

using namespace std;
using namespace Eigen;

void gridPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data        = new uint8_t[GLXYZ_SIZE];

    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++)
            {
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
                GridNodeMap[i][j][k]->occupancy = & data[i * GLYZ_SIZE + j * GLZ_SIZE + k];
            }
        }
    }
}

void gridPathFinder::resetMap()
{   
    //ROS_WARN("expandedNodes size : %d", expandedNodes.size());
    for(auto tmpPtr:expandedNodes)
    {
        //tmpPtr->occupancy = 0; 
        tmpPtr->id = 0;
        tmpPtr->cameFrom = NULL;
        tmpPtr->gScore = inf;
        tmpPtr->fScore = inf;
    }

    for(auto ptr:openSet)
    {   
        //tmpPtr->occupancy = 0; 
        GridNodePtr tmpPtr = ptr.second;
        tmpPtr->id = 0;
        tmpPtr->cameFrom = NULL;
        tmpPtr->gScore = inf;
        tmpPtr->fScore = inf;
    }

    expandedNodes.clear();
    //ROS_WARN("local map reset finish");
}

inline void gridPathFinder::coord2gridIndexFast(const double x, const double y, const double z, int & id_x, int & id_y, int & id_z)
{
    id_x = static_cast<int>( (x - gl_xl) * inv_resolution);
    id_y = static_cast<int>( (y - gl_yl) * inv_resolution);
    id_z = static_cast<int>( (z - gl_zl) * inv_resolution);      
}

void gridPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    tmp_id_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    tmp_id_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    tmp_id_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[tmp_id_x * GLYZ_SIZE + tmp_id_y * GLZ_SIZE + tmp_id_z] = 1;
}

void gridPathFinder::setObs(const Eigen::Vector3d coord)
{   
    //int tmp_id_x, tmp_id_y, tmp_id_z;
    coord2gridIndexFast(coord(0), coord(1), coord(2), tmp_id_x, tmp_id_y, tmp_id_z);
    if (tmp_id_x >= 0 && tmp_id_y >= 0 && tmp_id_z >= 0 && tmp_id_x < GLX_SIZE && tmp_id_y < GLY_SIZE && tmp_id_z < GLZ_SIZE)
    {
        data[tmp_id_x * GLYZ_SIZE + tmp_id_y * GLZ_SIZE + tmp_id_z] = 1;
    }
}

void gridPathFinder::setObs(const Eigen::Vector3i index)
{   
    if (index(0) >= 0 && index(1) >= 0 && index(2) >= 0 && index(0) < GLX_SIZE && index(1) < GLY_SIZE && index(2) < GLZ_SIZE)
    {
        data[index(0) * GLYZ_SIZE + index(1) * GLZ_SIZE + index(2)] = 1;
    }
}


double gridPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double gridPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double gridPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{   
    return (node2->index - node1->index).norm();
}

double gridPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker * getDiagHeu(node1, node2);
}

vector<GridNodePtr> gridPathFinder::retrievePath(GridNodePtr current)
{   
    vector<GridNodePtr> path;
    path.push_back(current);

    while(current->cameFrom != NULL)
    {
        current = current -> cameFrom;
        path.push_back(current);
    }

    return path;
}

vector<Vector3d> gridPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++)
            {   
                if(GridNodeMap[i][j][k]->id != 0)
                //if(GridNodeMap[i][j][k]->id == -1)
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

void gridPathFinder::AstarSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);

    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    openSet.clear();

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr, endPtr);
    startPtr -> id = 1; //put start node in open set
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while ( !openSet.empty() )
    {   
        num_iter ++;
        current = openSet.begin() -> second;

        if(current->index(0) == endPtr->index(0)
        && current->index(1) == endPtr->index(1)
        && current->index(2) == endPtr->index(2) )
        {
            /*ROS_WARN("[Astar]Reach goal..");
            cout << "total number of iteration used in Astar: " << num_iter  << endl;*/
            ros::Time time_2 = ros::Time::now();
            if((time_2 - time_1).toSec() > 0.1)
                ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath = retrievePath(current);
            return;
        }         
        openSet.erase(openSet.begin());
        current -> id = -1; //move current node from open set to closed set.
        expandedNodes.push_back(current);

        for(int dx = -1; dx < 2; dx++)
            for(int dy = -1; dy < 2; dy++)
                for(int dz = -1; dz < 2; dz++)
                {
                    if(dx == 0 && dy == 0 && dz ==0)
                        continue; 

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current -> index)(0) + dx;
                    neighborIdx(1) = (current -> index)(1) + dy;
                    neighborIdx(2) = (current -> index)(2) + dz;

                    if(    neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE
                        || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE
                        || neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE){
                        continue;
                    }

                    neighborPtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];

                    if(*(neighborPtr -> occupancy) == 1){
                        continue;
                    }

                    if(neighborPtr -> id == -1){
                        continue; //in closed set.
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    
                    tentative_gScore = current -> gScore + static_cost; 

                    if(neighborPtr -> id != 1){
                        //discover a new node
                        neighborPtr -> id        = 1;
                        neighborPtr -> cameFrom  = current;
                        neighborPtr -> gScore    = tentative_gScore;
                        neighborPtr -> fScore    = neighborPtr -> gScore + getHeu(neighborPtr, endPtr); 
                        neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                        continue;
                    }
                    else if(tentative_gScore <= neighborPtr-> gScore){ //in open set and need update
                        neighborPtr -> cameFrom = current;
                        neighborPtr -> gScore = tentative_gScore;
                        neighborPtr -> fScore = tentative_gScore + getHeu(neighborPtr, endPtr); 
                        openSet.erase(neighborPtr -> nodeMapIt);
                        neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                    }
                        
                }
    }

    ros::Time time_2 = ros::Time::now();

    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
}

vector<Vector3d> gridPathFinder::getPath()
{   
    vector<Vector3d> path;

    for(auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());
    return path;
}

void gridPathFinder::resetPath()
{
    gridPath.clear();
}