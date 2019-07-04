#ifndef _POLY_UTILS_H_
#define _POLY_UTILS_H_

#include <Eigen/Eigen>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include <quickhull/QuickHull.hpp>
#include <decomp_ros_msgs/Polyhedron.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
/*#include <polyhedron_generator/cluster_server.cuh>*/
#include <polyhedron_generator/cluster_server_cpu.h>

#include "glpk.h"
#include "data_type.h"
#include "eigen-cdd/Polyhedron.h"

using namespace quickhull;
using namespace std;
using namespace Eigen;

class polyhedronGenerator
{

private:
    cudaPolytopeGeneration * _cuda_polytope_generator;

    decomp_cvx_space::FlightCorridor corridor;
    double _resolution, _inv_resolution;
    int _max_x_id, _max_y_id, _max_z_id, _max_inf_iter, _max_clu_iter;
    Vector3d _map_lower, _map_upper;
    bool has_map;
    int poly_id = 0;
    
    Vector3d index2Coord(Vector3i index);
    Vector3d index2Coord(int id_x, int id_y, int id_z);
    Vector3i coord2Index(Vector3d coord);
    void coord2Index(int & id_x, int & id_y, int & id_z, const double & pt_x, const double & pt_y, const double & pt_z);
    bool isOutsidePolytope( const Vector3d & cur_coord, const decomp_cvx_space::Polytope & polytope);
    bool isInsidePolytope( const Vector3d & cur_coord, const decomp_cvx_space::Polytope & polytope);
    bool isOutsideLatestPolytope( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor);
    bool isInsideLastSecondPolytope( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor);
    bool isOutsideCorridor( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor);
    bool isInsideLatestPolytope( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor);
    bool isInsideCorridor( const Vector3d & cur_coord, const decomp_cvx_space::FlightCorridor & corridor);
    MatrixXd getVerticesPlane( const pair<MatrixXd, VectorXd> hrep, const MatrixXd & vertices3D);
    pair<decomp_ros_msgs::Polyhedron, decomp_cvx_space::Polytope> polyHrep2Utils(const pair<MatrixXd, VectorXd> & hrep, const MatrixXd & cur_vertices, bool is_degenarate);
    void getVoxelVertex( vector<Vector3d> & coordCorners, const vector<Vector3d> & coords);
    Vector3d round2Voxel(const Vector3d & coord);
    bool checkDegeneratePoly(const vector<int> & cluster_x_idx, const vector<int> & cluster_y_idx, const vector<int> & cluster_z_idx);
    pair <MatrixXd, bool> getConvexPoly( Vector3d seed_pt );
    pair <MatrixXd, bool> getConvexPoly( vector<Vector3d> seed_pts );

public:
    polyhedronGenerator(){};
    ~polyhedronGenerator(){};
    
    int corridorIncreGeneration (const vector<Vector3d> & coordSet, decomp_ros_msgs::PolyhedronArray & poly_array_msg);
    int corridorInsertGeneration(const vector<Vector3d> & coordSet, decomp_ros_msgs::PolyhedronArray & poly_array_msg);
    
    void corridorGeneration(const vector<Vector3d> & gridPath, decomp_ros_msgs::PolyhedronArray & poly_array_msg);

    void initialize(bool is_gpu_on_stage_1, bool is_gpu_on_stage_2, bool is_cluster_on,
        int max_x_id_, int max_y_id_, int max_z_id_, Vector3d map_lower_, Vector3d map_upper_, double resolution_, double inv_resolution_, int max_inf_iter_, int max_clu_iter_)
    {
        _resolution     = resolution_;
        _inv_resolution = inv_resolution_;

        _map_lower = map_lower_;
        _map_upper = map_upper_;

        _max_x_id = max_x_id_;
        _max_y_id = max_y_id_;
        _max_z_id = max_z_id_;
        _max_inf_iter = max_inf_iter_;
        _max_clu_iter = max_clu_iter_;

        has_map = false;

        _cuda_polytope_generator = new cudaPolytopeGeneration();
        _cuda_polytope_generator->paramSet( is_gpu_on_stage_1, is_gpu_on_stage_2, is_cluster_on, _max_x_id,  _max_y_id, _max_z_id, _resolution, _max_inf_iter, _max_clu_iter );
    };

    void setObs(Vector3i index)
    {
        _cuda_polytope_generator->setObs(index(0), index(1), index(2));
    };

    void finishMap()
    {
        _cuda_polytope_generator->mapUpload();  
        has_map = true;
    };

    void resetMap()
    {
        _cuda_polytope_generator->mapClear();  
        has_map = true;
    };

    Vector3d roundCoord(Vector3d coord)
    {
        return index2Coord(coord2Index(coord));
    };

    decomp_cvx_space::FlightCorridor getCorridor(){return corridor;};

    void reset()
    {
        poly_id = 0;
        corridor.polyhedrons.clear();
    };

};


#endif