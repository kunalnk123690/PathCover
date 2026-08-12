#ifndef POLYTOPE_UTILS_H
#define POLYTOPE_UTILS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <stdexcept>

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullPoints.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullHyperplane.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"

using namespace std;

namespace Geometry {


    template <typename T, int dim>
    inline void con2vert(const Eigen::Matrix<T, -1, dim> &A_column_major, 
                         const Eigen::Matrix<T, -1, 1> &b, 
                         const Eigen::Matrix<T, dim, 1> &seed, 
                         std::vector<Eigen::Matrix<T, dim, 1>> &Vertices) {
        Eigen::Matrix<T, -1, dim, Eigen::RowMajor> A = A_column_major;

        // Ensure seed is strictly inside the polytope by checking A*seed < b
        Eigen::Matrix<T, -1, 1> check = b - A * seed;
        for (int i = 0; i < check.size(); ++i) {
            if (check(i) <= 1e-8) {  // Allow small tolerance
                throw std::invalid_argument("Seed point is not strictly inside the polytope. Please provide a valid seed.");
            }
        }
        // Adjust b by subtracting A * seed
        Eigen::Matrix<T, -1, 1> b_adjusted = b - A * seed;

        // Normalize A by dividing rows by the corresponding elements of b
        Eigen::Matrix<T, -1, -1, Eigen::RowMajor> D = A.array().colwise() / b_adjusted.array();

        // 1. Initialize Qhull
        orgQhull::Qhull qhull;

        // "Qt" = triangulated output, "n" = compute normals
        qhull.runQhull("", dim, D.rows(), D.data(), "Qt n");        

        // Step 6: Extract vertices from the dual (which are facets in primal)
        // Each facet of the convex hull corresponds to a vertex of the polytope
        orgQhull::QhullFacetList facets = qhull.facetList();

        for (auto facet = facets.begin(); facet != facets.end(); ++facet) {
            // if (facet->isUpperDelaunay()) {
            //     continue; // Skip upper Delaunay facets
            // }

            // Get the hyperplane equation of this facet: n·x + offset = 0
            orgQhull::QhullHyperplane hyperplane = facet->hyperplane();

            // The vertex in the primal is at: -n / offset + seed
            T offset = hyperplane.offset();

            if (std::abs(offset) < 1e-10) {
                continue; // Skip facets through origin (unbounded direction)
            }

            Eigen::Matrix<T, dim, 1> vertex;
            for (int j = 0; j < dim; ++j) {
                vertex(j) = -hyperplane[j] / offset + seed(j);
            }

            // Verify vertex satisfies all constraints (numerical check)
            Eigen::Matrix<T, -1, 1> check = b - A * vertex;
            bool valid = true;
            for (int i = 0; i < b.size(); ++i) {
                if (check(i) < -1e-8) {  // Small tolerance for numerical errors
                    valid = false;
                    break;
                }
            }

            if (valid) {
                Vertices.push_back(vertex);
            }
        }

    }



    template <typename T, int dim>
    inline void computeMesh(const std::vector<Eigen::Matrix<T, dim, 1>> &Points,
                            std::vector<std::vector<Eigen::Matrix<T, dim, 1>>> &mesh) {
        static_assert(dim == 2 || dim == 3, "Dimension must be 2 or 3");

        int num_points = Points.size();

        if constexpr (dim == 2) {
            // A planar polytope's vertex set is already a convex polygon, so a
            // Delaunay triangulation is overkill (and qhull's 'd' mode needs
            // upper-facet filtering to be used correctly). Sorting the
            // vertices by angle about their centroid recovers the polygon
            // boundary exactly, and a triangle fan from the first vertex
            // tiles a convex polygon with no degenerate triangles.
            if (num_points < 3) {
                return;
            }

            Eigen::Matrix<T, 2, 1> centroid = Eigen::Matrix<T, 2, 1>::Zero();
            for (const auto &p : Points) {
                centroid += p;
            }
            centroid /= static_cast<T>(num_points);

            std::vector<Eigen::Matrix<T, 2, 1>> ordered(Points);
            std::sort(ordered.begin(), ordered.end(),
                      [&centroid](const Eigen::Matrix<T, 2, 1> &a,
                                  const Eigen::Matrix<T, 2, 1> &b) {
                          return std::atan2(a(1) - centroid(1), a(0) - centroid(0)) <
                                 std::atan2(b(1) - centroid(1), b(0) - centroid(0));
                      });

            for (int i = 1; i + 1 < num_points; ++i) {
                mesh.push_back({ordered[0], ordered[i], ordered[i + 1]});
            }
        }
        else {
            // Map the points vector into an Eigen matrix
            Eigen::Map<const Eigen::Matrix<T, dim, -1>> point_matrix(
                reinterpret_cast<const T *>(Points.data()), dim, num_points);
            Eigen::Matrix<double, -1, dim, Eigen::RowMajor> points_transpose = point_matrix.transpose().template cast<double>();

            orgQhull::Qhull qhull;
            qhull.runQhull("", dim, num_points, points_transpose.data(), "Qt");

            // Process the facets of the convex hull
            for (const auto &facet : qhull.facetList()) {
                if (facet.isGood()) {
                    std::vector<Eigen::Matrix<T, dim, 1>> simplex;
                    std::transform(facet.vertices().begin(), facet.vertices().end(), std::back_inserter(simplex),
                                   [](const auto &vertex) {
                                       const auto &point = vertex.point();
                                       return Eigen::Matrix<T, dim, 1>(static_cast<T>(point[0]),
                                                                       static_cast<T>(point[1]),
                                                                       static_cast<T>(point[2]));
                                   });
                    mesh.push_back(simplex);
                }
            }
        }
    }


    template <typename T, int dim>
    inline void constructMesh(std::vector<Eigen::Matrix<T, -1, dim>>& A, 
                              std::vector<Eigen::Matrix<T, -1, 1>>& b,
                              std::vector<Eigen::Matrix<T, dim, 1>>& seeds, 
                              std::vector<std::vector<Eigen::Matrix<T, dim, 1>>>& mesh, 
                              int horizon) {
            
        std::vector<Eigen::Matrix<T, dim, 1>> vertices_;
        std::vector<std::vector<Eigen::Matrix<T, dim, 1>>> mesh_;

        int limit = std::min(horizon, static_cast<int>(A.size()));
        
        for(int i = 0; i < limit; i++) {
            vertices_.clear();
            mesh_.clear();
            // std::cout << "Constructing polyhedron ...\n";
            // std::cout << "A size: " << A[i] << std::endl;
            // std::cout << "b size: " << b[i].transpose() << std::endl;
            // std::cout << "seed: " << seeds[i].transpose() << std::endl;
            con2vert<T, dim>(A[i], b[i], seeds[i], vertices_);
            // if (vertices_.size() < 4) {
            //     std::cout << "Vertices size: " << vertices_.size() << "\n";
            //     std::cout << "A:\n" << A[i] << "\n";
            //     std::cout << "b:\n" << b[i].transpose() << "\n";
            //     std::cout << "seed:\n" << seeds[i].transpose() << "\n";
            //     std::cout << "Failed to compute vertices for polyhedron\n";
            //     continue;
            // }
            computeMesh<T, dim>(vertices_, mesh_);
            mesh.insert(mesh.end(), mesh_.begin(), mesh_.end());
            // std::cout << "Constructed polyhedron " << i+1 << " / " << limit << std::endl;
        }

    }

}


#endif // POLYTOPE_UTILS_H