#ifndef POLYTOPE_PUBLISHER_HPP
#define POLYTOPE_PUBLISHER_HPP

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry/polytope_utils.h"


template <typename T, int dim>
class PolyhedraPublisher {
    private:
        rclcpp::Node *node_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub_;

    public:
        visualization_msgs::msg::Marker meshMarker;
        visualization_msgs::msg::Marker edgeMarker;

    public:
        PolyhedraPublisher(rclcpp::Node *node, std::string meshTopic, std::string edgeTopic) : node_(node) {
            // Constructor
            meshPub_ = node_->create_publisher<visualization_msgs::msg::Marker>(meshTopic, 1000);
            edgePub_ = node_->create_publisher<visualization_msgs::msg::Marker>(edgeTopic, 1000);

            meshMarker.id = 0;
            meshMarker.header.stamp = node_->now();
            meshMarker.header.frame_id = "world";
            meshMarker.pose.orientation.w = 1.00;
            meshMarker.action = visualization_msgs::msg::Marker::ADD;
            meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            meshMarker.ns = "mesh";
            meshMarker.color.r = 0.00;
            meshMarker.color.g = 0.00;
            meshMarker.color.b = 1.00;
            meshMarker.color.a = 0.2;
            meshMarker.scale.x = 1.0;
            meshMarker.scale.y = 1.0;
            meshMarker.scale.z = 1.0;
            edgeMarker = meshMarker;
            edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
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
            geometry_msgs::msg::Point mesh_point;
            for (int i = 0; i < mesh.size(); i++) {
                for(int j = 0; j < mesh[i].size(); j++) {
                    Eigen::Map<Eigen::Matrix<double, dim, 1>>(&mesh_point.x) = mesh[i][j].template cast<double>();
                    meshMarker.points.push_back(mesh_point);
                }
            }

            edgeMarker.points.clear();
            geometry_msgs::msg::Point edge_point;
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

            meshPub_->publish(meshMarker);
            edgePub_->publish(edgeMarker);
        }

        ~PolyhedraPublisher(){};

};



#endif // POLYTOPE_PUBLISHER_HPP
