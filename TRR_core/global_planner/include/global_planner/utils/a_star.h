#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
   int id;        // 1--> open set, -1 --> closed set
   Eigen::Vector3d coord;
   Eigen::Vector3i index;
   
   double gScore, fScore;
   GridNodePtr cameFrom;
   std::multimap<double, GridNodePtr>::iterator nodeMapIt;
   uint8_t * occupancy; 

   GridNode(Eigen::Vector3i _index)
   {  
      id = 0;
      index = _index;
      
      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
   }

   GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
   {  
      id = 0;
      index = _index;
      coord = _coord;

      gScore = inf;
      fScore = inf;
      cameFrom = NULL;
   }

   GridNode(){};
   
   ~GridNode(){};
};

class gridPathFinder
{
	private:
		inline void coord2gridIndexFast(const double x, const double y, const double z, int & id_x, int & id_y, int & id_z);
		
		double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
		double getManhHeu(GridNodePtr node1, GridNodePtr node2);
		double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
		double getHeu(GridNodePtr node1, GridNodePtr node2);

		std::vector<GridNodePtr> retrievePath(GridNodePtr current);

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
		double tie_breaker = 1.0 + 1.0 / 10000;

		std::vector<GridNodePtr> expandedNodes;
		std::vector<GridNodePtr> gridPath;

		std::vector<GridNodePtr> endPtrList;
    	std::vector<double> globalHeuList;

		int tmp_id_x, tmp_id_y, tmp_id_z;
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		uint8_t * data;

		GridNodePtr *** GridNodeMap;
		std::multimap<double, GridNodePtr> openSet;

	public:
		gridPathFinder( int max_x_id, int max_y_id, int max_z_id )
		{	
			// size of a big big global grid map
			GLX_SIZE = max_x_id;
			GLY_SIZE = max_y_id;
			GLZ_SIZE = max_z_id;
			GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
			GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
		};

		gridPathFinder(){};
		~gridPathFinder(){};

		void setObs(const double coord_x, const double coord_y, const double coord_z);
		void setObs(const Eigen::Vector3d coord);
		void setObs(const Eigen::Vector3i index);

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u);
		void AstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

		void resetMap();
		void resetPath();

		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();

		GridNodePtr getGridNode(Eigen::Vector3d coord)
		{
			Eigen::Vector3i index = coord2gridIndex(coord);			
			return GridNodeMap[index(0)][index(1)][index(2)];
		}

		GridNodePtr getGridNode(double x, double y, double z)
		{
			Eigen::Vector3d coord(x, y, z);
			Eigen::Vector3i index = coord2gridIndex(coord);			
			return GridNodeMap[index(0)][index(1)][index(2)];
		}

		GridNodePtr getGridNode(int id_x, int id_y, int id_z)
		{
			return GridNodeMap[id_x][id_y][id_z];
		}

		GridNodePtr getGridNode(Eigen::Vector3i index)
		{
			return GridNodeMap[index(0)][index(1)][index(2)];
		}

		inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i index) const
		{
		    Eigen::Vector3d pt;

		    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
		    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
		    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

		    return pt;
		};

		inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d pt) const
		{
		    Eigen::Vector3i idx;
		    idx <<  std::min( std::max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
		            std::min( std::max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
		            std::min( std::max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);      		    
		  
		    return idx;
		};

		inline uint8_t IndexQuery( const Eigen::Vector3i index) const
		{   
			if (index(0) >= 0 && index(1) >= 0 && index(2) >= 0 && index(0) < GLX_SIZE && index(1) < GLY_SIZE && index(2) < GLZ_SIZE)
			    //return data[index(0)][index(1)][index(2)];
			    return data[index(0) * GLYZ_SIZE + index(1) * GLZ_SIZE + index(2)];
			else
				return 1.0;
		};

		inline uint8_t IndexQueryFast( const Eigen::Vector3i & index) const
		{   
			return data[index(0) * GLYZ_SIZE + index(1) * GLZ_SIZE + index(2)];
		};

		inline uint8_t IndexQuery( const int & index_x, const int & index_y, const int & index_z) const
		{      
		    //return data[index_x][index_y][index_z];
		    return data[index_x * GLYZ_SIZE + index_y * GLZ_SIZE + index_z];
		};

		inline uint8_t CoordQuery(const Eigen::Vector3d & coord) const
		{   
		    Eigen::Vector3i index = coord2gridIndex(coord);

		    if (index(0) >= 0 && index(1) >= 0 && index(2) >= 0 && index(0) < GLX_SIZE && index(1) < GLY_SIZE && index(2) < GLZ_SIZE)
		    	return data[index(0) * GLYZ_SIZE + index(1) * GLZ_SIZE + index(2)];
			    //return data[index(0)][index(1)][index(2)];
			else
				return 1.0;
		};

		inline uint8_t CoordQuery( const double & pt_x, const double & pt_y, const double & pt_z ) const
		{   
		    Eigen::Vector3d coord(pt_x, pt_y, pt_z);
		    Eigen::Vector3i index = coord2gridIndex(coord);

		    if (index(0) >= 0 && index(1) >= 0 && index(2) >= 0 && index(0) < GLX_SIZE && index(1) < GLY_SIZE && index(2) < GLZ_SIZE)
		    	return data[index(0) * GLYZ_SIZE + index(1) * GLZ_SIZE + index(2)];
			    //return data[index(0)][index(1)][index(2)];
			else
				return 1.0;
		};
};