#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

// Shared visualizer for both planar and spatial planners. For 2D data, z_ is
// used as the display height; 3D data retains its own z coordinate.
class Visualizer {
    private:
        using VectorDim = Eigen::Matrix<double, PATHCOVER_DIM, 1>;

        inline double markerHeight(const VectorDim &point) const {
            if constexpr (PATHCOVER_DIM == 2) {
                return z_;
            } else {
                return point(2);
            }
        }

        ros::NodeHandle nh;
        double z_;

        // These are publishers for path, waypoints on the trajectory,
        // the entire trajectory, the mesh of free-space polytopes,
        // the edge of free-space polytopes, and spheres for safety radius
        ros::Publisher routePub;
        ros::Publisher seedPub;
        ros::Publisher wayPointsPub;
        ros::Publisher waySeedsPub;
        ros::Publisher startPub;
        ros::Publisher goalPub;


    public:
        Visualizer(ros::NodeHandle &nh_, double vizHeight = 0.0) : nh(nh_), z_(vizHeight) {
            routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
            seedPub = nh.advertise<visualization_msgs::Marker>("/visualizer/seeds", 10);
            wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
            waySeedsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/wayseeds", 10);
            startPub = nh.advertise<visualization_msgs::Marker>("/visualizer/start", 1);
            goalPub = nh.advertise<visualization_msgs::Marker>("/visualizer/goal", 1);
        }

        inline void setVizHeight(double vizHeight) { z_ = vizHeight; }


        // Visualize path
        inline void visualize_path(const std::vector<VectorDim> &route) {
            visualization_msgs::Marker routeMarker, wayPointsMarker;

            routeMarker.id = 0;
            routeMarker.type = visualization_msgs::Marker::LINE_LIST;
            routeMarker.header.stamp = ros::Time::now();
            routeMarker.header.frame_id = "world";
            routeMarker.pose.orientation.w = 1.00;
            routeMarker.action = visualization_msgs::Marker::ADD;
            routeMarker.ns = "route";
            routeMarker.color.r = 1.00;
            routeMarker.color.g = 0.00;
            routeMarker.color.b = 0.00;
            routeMarker.color.a = 1.00;
            routeMarker.scale.x = 0.05;

            wayPointsMarker = routeMarker;
            wayPointsMarker.id = -wayPointsMarker.id - 1;
            wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
            wayPointsMarker.ns = "waypoints";
            wayPointsMarker.color.r = 0.00;
            wayPointsMarker.color.g = 0.00;
            wayPointsMarker.color.b = 1.00;
            wayPointsMarker.scale.x = 0.15;
            wayPointsMarker.scale.y = 0.15;
            wayPointsMarker.scale.z = 0.15;


            if (route.size() > 0) {
                bool first = true;
                VectorDim last;
                for (auto it : route) {
                    if (first) {
                        first = false;
                        last = it;
                        continue;
                    }
                    geometry_msgs::Point point;

                    point.x = last(0);
                    point.y = last(1);
                    point.z = markerHeight(last);
                    routeMarker.points.push_back(point);
                    point.x = it(0);
                    point.y = it(1);
                    point.z = markerHeight(it);
                    routeMarker.points.push_back(point);
                    last = it;
                }

                routePub.publish(routeMarker);
            }

            for (int i = 1; i + 1 < static_cast<int>(route.size()); i++) {
                geometry_msgs::Point point;
                point.x = route[i](0);
                point.y = route[i](1);
                point.z = markerHeight(route[i]);
                wayPointsMarker.points.push_back(point);
            }
            wayPointsPub.publish(wayPointsMarker);
        }


        // Visualize seeds
        inline void visualize_seeds(const std::vector<VectorDim> &route) {
            visualization_msgs::Marker routeMarker, wayPointsMarker;

            routeMarker.id = 0;
            routeMarker.type = visualization_msgs::Marker::LINE_LIST;
            routeMarker.header.stamp = ros::Time::now();
            routeMarker.header.frame_id = "world";
            routeMarker.pose.orientation.w = 1.00;
            routeMarker.action = visualization_msgs::Marker::ADD;
            routeMarker.ns = "route";
            routeMarker.color.r = 1.00;
            routeMarker.color.g = 0.00;
            routeMarker.color.b = 1.00;
            routeMarker.color.a = 1.00;
            routeMarker.scale.x = 0.05;

            wayPointsMarker = routeMarker;
            wayPointsMarker.id = -wayPointsMarker.id - 1;
            wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
            wayPointsMarker.ns = "waypoints";
            wayPointsMarker.color.r = 0.00;
            wayPointsMarker.color.g = 0.00;
            wayPointsMarker.color.b = 0.00;
            wayPointsMarker.scale.x = 0.20;
            wayPointsMarker.scale.y = 0.20;
            wayPointsMarker.scale.z = 0.20;


            if (route.size() > 0) {
                bool first = true;
                VectorDim last;
                for (auto it : route) {
                    if (first) {
                        first = false;
                        last = it;
                        continue;
                    }
                    geometry_msgs::Point point;

                    point.x = last(0);
                    point.y = last(1);
                    point.z = markerHeight(last);
                    routeMarker.points.push_back(point);
                    point.x = it(0);
                    point.y = it(1);
                    point.z = markerHeight(it);
                    routeMarker.points.push_back(point);
                    last = it;
                }

                seedPub.publish(routeMarker);
            }

            for (int i = 1; i < static_cast<int>(route.size()); i++) {
                geometry_msgs::Point point;
                point.x = route[i](0);
                point.y = route[i](1);
                point.z = markerHeight(route[i]);
                wayPointsMarker.points.push_back(point);
            }
            waySeedsPub.publish(wayPointsMarker);

        }



        // Visualize all spheres with centers sphs and the same radius
        inline void visualizeGoal(const VectorDim &center,
                                  const double &radius,
                                  const Eigen::Vector4d &color) {
            visualization_msgs::Marker sphereMarkers, sphereDeleter;

            sphereMarkers.id = 0;
            sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
            sphereMarkers.header.stamp = ros::Time::now();
            sphereMarkers.header.frame_id = "world";
            sphereMarkers.pose.orientation.w = 1.00;
            sphereMarkers.action = visualization_msgs::Marker::ADD;
            sphereMarkers.ns = "spheres";
            sphereMarkers.color.r = color(0);
            sphereMarkers.color.g = color(1);
            sphereMarkers.color.b = color(2);
            sphereMarkers.color.a = color(3);
            sphereMarkers.scale.x = radius;
            sphereMarkers.scale.y = radius;
            sphereMarkers.scale.z = radius;

            sphereDeleter = sphereMarkers;
            sphereDeleter.action = visualization_msgs::Marker::DELETE;

            geometry_msgs::Point point;
            point.x = center(0);
            point.y = center(1);
            point.z = markerHeight(center);
            sphereMarkers.points.push_back(point);

            goalPub.publish(sphereDeleter);
            goalPub.publish(sphereMarkers);
        }


        inline void visualizeStart(const VectorDim &center,
                                   const double &radius,
                                   const Eigen::Vector4d &color) {
            visualization_msgs::Marker sphereMarkers, sphereDeleter;

            sphereMarkers.id = 0;
            sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
            sphereMarkers.header.stamp = ros::Time::now();
            sphereMarkers.header.frame_id = "world";
            sphereMarkers.pose.orientation.w = 1.00;
            sphereMarkers.action = visualization_msgs::Marker::ADD;
            sphereMarkers.ns = "StartGoal";
            sphereMarkers.color.r = color(0);
            sphereMarkers.color.g = color(1);
            sphereMarkers.color.b = color(2);
            sphereMarkers.color.a = color(3);
            sphereMarkers.scale.x = radius;
            sphereMarkers.scale.y = radius;
            sphereMarkers.scale.z = radius;

            sphereDeleter = sphereMarkers;
            sphereDeleter.action = visualization_msgs::Marker::DELETE;

            geometry_msgs::Point point;
            point.x = center(0);
            point.y = center(1);
            point.z = markerHeight(center);
            sphereMarkers.points.push_back(point);

            startPub.publish(sphereDeleter);
            startPub.publish(sphereMarkers);
        }

};

#endif // VISUALIZER_HPP
