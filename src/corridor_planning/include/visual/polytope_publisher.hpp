#ifndef POLYTOPE_PUBLISHER_HPP
#define POLYTOPE_PUBLISHER_HPP

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry/polytope_utils.h"


template <typename T, int dim>
class PolyhedraPublisher {
    private:
        ros::Publisher meshPub_;
        ros::Publisher edgePub_;
        // World z the polytopes are drawn at. Only used when dim == 2, where
        // the polytope itself carries no height.
        double vizHeight_;

    public:
        visualization_msgs::Marker meshMarker;
        visualization_msgs::Marker edgeMarker;

    public:
        PolyhedraPublisher(ros::NodeHandle &nh, std::string meshTopic, std::string edgeTopic,
                           double vizHeight = 0.0)
            : vizHeight_(vizHeight) {
            // Constructor
            meshPub_ = nh.advertise<visualization_msgs::Marker>(meshTopic, 1000);
            edgePub_ = nh.advertise<visualization_msgs::Marker>(edgeTopic, 1000);
            
            meshMarker.id = 0;
            meshMarker.header.stamp = ros::Time::now();
            meshMarker.header.frame_id = "world";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.action = visualization_msgs::Marker::ADD;
            meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";
            meshMarker.color.r = 0.00;
            meshMarker.color.g = 0.00;
            meshMarker.color.b = 1.00;
            meshMarker.color.a = 0.2;
            meshMarker.scale.x = 1.0;
            meshMarker.scale.y = 1.0;
            meshMarker.scale.z = 1.0;
            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
            edgeMarker.ns = "edge";
            edgeMarker.color.r = 0.00;
            edgeMarker.color.g = 1.00;
            edgeMarker.color.b = 1.00;
            edgeMarker.color.a = 1.00;
            edgeMarker.scale.x = 0.02;

        };

        
        inline void publishMesh(std::vector<Eigen::Matrix<T, -1, dim>> &A, 
                                std::vector<Eigen::Matrix<T, -1, 1>> &b,
                                std::vector<Eigen::Matrix<T, dim, 1>> &seeds, 
                                int horizon) {
            
            std::vector<std::vector<Eigen::Matrix<T, dim, 1>>> mesh;
            Geometry::constructMesh<T, dim>(A, b, seeds, mesh, horizon);
            meshMarker.points.clear();
            geometry_msgs::Point mesh_point;
            // For dim == 2 the Eigen::Map below only writes x and y, so z has
            // to be set here; for dim == 3 it is overwritten by the map.
            mesh_point.z = vizHeight_;
            for (int i = 0; i < mesh.size(); i++) {
                for(int j = 0; j < mesh[i].size(); j++) {
                    Eigen::Map<Eigen::Matrix<double, dim, 1>>(&mesh_point.x) = mesh[i][j].template cast<double>();
                    meshMarker.points.push_back(mesh_point);
                }
            }

            edgeMarker.points.clear();
            geometry_msgs::Point edge_point;
            edge_point.z = vizHeight_;
            for (int i = 0; i < mesh.size(); i++) {
                for (int j = 0; j < mesh[i].size(); j++) {
                    Eigen::Map<Eigen::Matrix<double, dim, 1>>(&edge_point.x) = mesh[i][j].template cast<double>();
                    edgeMarker.points.push_back(edge_point);
                }
                for (int j = 0; j < mesh[i].size(); j++) {
                    Eigen::Map<Eigen::Matrix<double, dim, 1>>(&edge_point.x) = mesh[i][j].template cast<double>();
                    edgeMarker.points.push_back(edge_point);
                }
            }

            meshPub_.publish(meshMarker);
            edgePub_.publish(edgeMarker);
        }

        ~PolyhedraPublisher(){};

};



#endif // POLYTOPE_PUBLISHER_HPP