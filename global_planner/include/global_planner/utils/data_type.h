#ifndef _DATA_TYPE_
#define _DATA_TYPE_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

#define inf 999999.0

namespace decomp_cvx_space
{
   struct Polytope;
   typedef Polytope* PolytopePtr;

   struct Cube
   {     
         //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;   // the 8 vertex of a cube 
         Eigen::MatrixXd vertex;
         Eigen::Vector3d center; // the center of the cube
         bool valid;    // indicates whether this cube should be deleted

         double t; // time allocated to this cube
         std::vector< std::pair<double, double> > box;
   /*
              P4------------P3 
              /|           /|              ^
             / |          / |              | z
           P1--|---------P2 |              |
            |  P8--------|--p7             |
            | /          | /               /--------> y
            |/           |/               /  
           P5------------P6              / x
   */                                                                                 

         // create a cube using 8 vertex and the center point
         Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
         {
               vertex = vertex_;
               center = center_;
               valid = true;
               t = 0.0;
               box.resize(3);
         }

         // create a inscribe cube of a ball using the center point and the radius of the ball
         void setVertex( Eigen::MatrixXd vertex_, double resolution_)
         {     
               vertex = vertex_;
               vertex(0,1) -= resolution_ / 2.0;
               vertex(3,1) -= resolution_ / 2.0;
               vertex(4,1) -= resolution_ / 2.0;
               vertex(7,1) -= resolution_ / 2.0;

               vertex(1,1) += resolution_ / 2.0;
               vertex(2,1) += resolution_ / 2.0;
               vertex(5,1) += resolution_ / 2.0;
               vertex(6,1) += resolution_ / 2.0;

               vertex(0,0) += resolution_ / 2.0;
               vertex(1,0) += resolution_ / 2.0;
               vertex(4,0) += resolution_ / 2.0;
               vertex(5,0) += resolution_ / 2.0;

               vertex(2,0) -= resolution_ / 2.0;
               vertex(3,0) -= resolution_ / 2.0;
               vertex(6,0) -= resolution_ / 2.0;
               vertex(7,0) -= resolution_ / 2.0;

               vertex(0,2) += resolution_ / 2.0;
               vertex(1,2) += resolution_ / 2.0;
               vertex(2,2) += resolution_ / 2.0;
               vertex(3,2) += resolution_ / 2.0;

               vertex(4,2) -= resolution_ / 2.0;
               vertex(5,2) -= resolution_ / 2.0;
               vertex(6,2) -= resolution_ / 2.0;
               vertex(7,2) -= resolution_ / 2.0;

               setBox();
         }
         
         void setBox()
         {
               box.clear();
               box.resize(3);

               for(int i = 0; i < 8; i++)
                  for(int j = 0; j < 3; j++)
                     if( std::fabs(vertex(i, j)) < 0.001 )
                         vertex(i, j) = 0.0;

               box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) );
               box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) );
               box[2] = std::make_pair( vertex(4, 2), vertex(1, 2) );
         }

         void printBox()
         {
               std::cout<<"center of the cube: \n"<<center<<std::endl;
               std::cout<<"bounding of the cube:"<<std::endl;
               std::cout<<box[0].first<<", "<<box[0].second<<std::endl;
               std::cout<<box[1].first<<", "<<box[1].second<<std::endl;
               std::cout<<box[2].first<<", "<<box[2].second<<std::endl;
         }

         Cube()
         {  
            center = Eigen::VectorXd::Zero(3);
            vertex = Eigen::MatrixXd::Zero(8, 3);

            valid = true;
            t = 0.0;
            box.resize(3);
         }

         ~Cube(){}
   };

   struct Polytope
   {     
         Eigen::Vector3d center; // the center of the polytope
         Eigen::Vector3d seed_coord; // the seed coordinate when generating the polytope
         Eigen::Vector3i seed_index; // the center of the polytope
         int coord_num;
         std::vector< Eigen::Vector4d > planes;                                                                         
         std::vector< Eigen::Vector3d > vertices;                                                                         
         
         double t; // time allocated to this polytope

         Polytope( std::vector< Eigen::Vector4d > planes_, Eigen::Vector3d center_)
         {
            planes = planes_;
            center = center_;
            t = 0.0;
         }

         void setPlanes( std::vector< Eigen::Vector4d > planes_ )
         {     
            planes = planes_;
         }
         
         void setCenter( Eigen::Vector3d center_ )
         {     
            center = center_;
         }

         void setSeed( Eigen::Vector3d seed_coord_ )
         {     
            seed_coord = seed_coord_;
         }

         void appendPlane( Eigen::Vector4d plane )
         {        
            //std::cout<<"append plane: \n"<<plane<<std::endl;
            planes.push_back(plane);
            //std::cout<<"planes size: "<<planes.size()<<std::endl;
         }

         void appendVertex( Eigen::Vector3d vertex )
         {        
            vertices.push_back(vertex);
         }

         void printPlanes()
         {  
            std::cout<<"check hyperplane functions: "<<std::endl;
            for(auto pl:planes)
               std::cout<<pl(0)<<", "<<pl(1)<<", "<<pl(2)<<", "<<pl(3)<<std::endl;
         }

         Polytope()
         { 
            planes.reserve(100);
            vertices.reserve(100);
            t = 0.0;
            coord_num = -1;
         }

         ~Polytope()
         {

         }
   };

   struct FlightCorridor
   {     
         std::vector< double >   durations;   // time duration allocated to each polytope
         std::vector< Polytope > polyhedrons; // geometric shape of each polytope                            
         double scale_factor;

         void appendPolytope( Polytope pltp )
         {     
            polyhedrons.push_back(pltp);
         }

         void appendTime( double t )
         {     
            durations.push_back(t);
         }

         void clearCorridor()
         {
            polyhedrons.clear();
            durations.clear();
         }

         void printCorridor()
         {  
            std::cout<<"check polytopes: "<<std::endl;
            for(auto pltp:polyhedrons)
               pltp.printPlanes();

            std::cout<<"check time segments: "<<std::endl;
            for(auto t:durations)
               std::cout<<t<<std::endl;
         }

         bool isEmpty()
         {
            if(polyhedrons.size() == 0)
               return true;
            else
               return false;
         }

         void clear()
         {
            durations.clear();
            polyhedrons.clear();
         }

         FlightCorridor()
         {  
            polyhedrons.reserve(100);
            durations.reserve(100);
            scale_factor = 1.0;
         }

         ~FlightCorridor(){}
   };
};

#endif