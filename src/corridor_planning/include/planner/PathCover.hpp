#ifndef PATHCOVER_HPP
#define PATHCOVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <libqhullcpp/Qhull.h>


namespace PathCover {

template<typename T, const int dim> // Uses qhull
inline void qConvexHull(Eigen::Matrix<T, -1, -1, Eigen::RowMajor> &Points, 
                        std::vector<int> &idx) {
    
    std::vector<double> points_mapped(Points.data(), Points.data() + Points.size());
    orgQhull::Qhull qhull = orgQhull::Qhull("", dim, Points.rows(), points_mapped.data(), "");
    
    idx.reserve(qhull.vertexList().size());
    for (auto vi = qhull.vertexList().begin(); vi != qhull.vertexList().end(); ++vi) {
      idx.push_back(vi->point().id());
    }
}    


template <typename T, int Dim>
inline void deflatePolyhedra(const Eigen::Matrix<T, -1, Dim> &A, 
                             Eigen::Matrix<T, -1, 1> &b,
                             T offset) {

    for (int i = 0; i < b.size(); ++i) {
        b(i) = b(i) - offset * A.row(i).norm();
    }
}


template<typename T, const int dim>
inline void noredund(Eigen::Matrix<T, -1, dim> &A, 
                     Eigen::Matrix<T, -1, 1> &b, 
                     const Eigen::Matrix<T, dim, 1> &seed) {

    // Ensure seed is strictly inside the polytope by checking A*seed < b
    Eigen::Matrix<T, -1, 1> check = b - A * seed;
    for (int i = 0; i < check.size(); ++i) {
        if (check(i) <= 1e-8) {  // Allow small tolerance
            std::cout << A << "\n" << b.transpose() << "\n" << seed.transpose() << "\n";
            throw std::invalid_argument("Seed point is not strictly inside the polytope. Redundancy removal failed!");
        }
    }                        
    
    // obtain dual polytope vertices
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> D = (A.array().colwise() / (b - A * seed).array());
    

    std::vector<int> idx;

    qConvexHull<T, dim>(D, idx);

    
    // Temporary matrices to store the rows corresponding to the convex hull
    Eigen::Matrix<T, -1, dim> An(idx.size(), A.cols());
    Eigen::Matrix<T, -1, 1> bn(idx.size());

    for (int i = 0; i < idx.size(); ++i) {
        An.row(i) = A.row(idx[i]);
        bn(i) = b(idx[i]);
    }

    A = An;
    b = bn;
}


// RISP (Randomized Iterative Space Partitioning), Algorithm 2 of the paper.
// For each randomly sampled obstacle point p, the cutting hyperplane is
//
//     a = p - y_seed,
//     b = a^T y_seed + (1 - alpha) * ||a||^2,
//
// which is Eq. (4) of the paper. The plane passes through the point
// alpha*y_seed + (1-alpha)*p and the open ball of radius (1-alpha)*||a||
// around y_seed is strictly contained in the half-space a^T x <= b.
// alpha must lie in (0, 1); smaller alpha -> larger polytope but less margin,
// larger alpha -> smaller polytope with a wider safety annulus around the seed.
template<typename T, const int dim>
inline void RISP(const std::vector<Eigen::Matrix<T, dim, 1>> &obstacle_pts, 
                 const Eigen::Matrix<T, dim, 1> &seed, 
                 const Eigen::Matrix<T, -1, dim> &A_bound, 
                 const Eigen::Matrix<T, -1, 1> &b_bound, 
                 Eigen::Matrix<T, -1, dim> &A, 
                 Eigen::Matrix<T, -1, 1> &b,
                 std::vector<int> &pts_remaining,
                 double offset = 0,
                 T alpha = T(0.01)) {
    assert(alpha > T(0) && alpha < T(1) && "RISP: alpha must lie in (0, 1)");

    std::vector<Eigen::Matrix<T, dim, 1>> obs_to_consider = obstacle_pts; // Copy for modification

    // Preallocate A and b with a conservative size
    A.conservativeResize(obstacle_pts.size(), dim);
    b.conservativeResize(obstacle_pts.size());

    // Clear and reserve pts_remaining
    pts_remaining.clear();
    pts_remaining.reserve(obstacle_pts.size());
    pts_remaining.push_back(static_cast<int>(obs_to_consider.size()));    

    // Start the loop
    Eigen::Matrix<T, dim, 1> xi, normal_vector;
    T b0;
    int k = 0;
    const T one_minus_alpha = T(1) - alpha;
    while (!obs_to_consider.empty()) {
        // Select a random obstacle point
        xi = obs_to_consider[rand() % obs_to_consider.size()];

        // Compute the normal vector and offset following Eq. (4):
        //   a = xi - seed,  b = a . seed + (1 - alpha) * ||a||^2.
        normal_vector = (xi - seed);
        b0 = normal_vector.dot(seed) + one_minus_alpha * normal_vector.squaredNorm();

        // Add the constraint to A and b
        A.row(k) = normal_vector.transpose();
        b(k) = b0;

        // Discard points strictly on the far side of the hyperplane (Alg. 2,
        // lines 7-9: remove q iff a^T q > b). With alpha > 0 the sampled xi
        // itself satisfies a^T xi - b = alpha*||a||^2 > 0, so it is removed
        // and the loop is guaranteed to terminate.
        auto it = std::remove_if(obs_to_consider.begin(), obs_to_consider.end(),
            [&normal_vector, b0](const Eigen::Matrix<T, dim, 1>& obs) {
                return normal_vector.dot(obs) > b0;
            });
        obs_to_consider.erase(it, obs_to_consider.end());

        // Record the number of remaining obstacle points after this separation step
        pts_remaining.push_back(static_cast<int>(obs_to_consider.size()));
                
        k++;
    }

    // Resize A and b to the actual number of constraints
    A.conservativeResize(k + A_bound.rows(), Eigen::NoChange);
    b.conservativeResize(k + b_bound.size());

    // Append A_bound and b_bound to A and b
    A.bottomRows(A_bound.rows()) = A_bound;
    b.tail(b_bound.size()) = b_bound;

    // Remove redundant constraints
    noredund<T, dim>(A, b, seed);
    if (offset > 0) {
        deflatePolyhedra<T, dim>(A, b, offset);
    }
}


template <typename T, int Dim>
inline Eigen::Matrix<T, Dim, 1> computePolyhedraIntersection(const Eigen::Matrix<T, -1, Dim> &A, 
                                                             const Eigen::Matrix<T, -1, 1> &b,
                                                             const Eigen::Matrix<T, Dim, 1> &p1, 
                                                             const Eigen::Matrix<T, Dim, 1> &p2) {
    T t_min = std::numeric_limits<T>::max();
    Eigen::Matrix<T, Dim, 1> intersection_point;

    Eigen::Matrix<T, Dim, 1> direction = p2 - p1;

    for (int i = 0; i < A.rows(); ++i) {
        T denom = A.row(i).dot(direction);
        if (std::abs(denom) < 1e-10) continue; // Skip parallel planes

        T t = (b(i) - A.row(i).dot(p1)) / denom;
        
        if (t >= 0.0 && t <= 1.0) { // Intersection must be within segment
            Eigen::Matrix<T, Dim, 1> candidate = p1 + t * direction;

            if (t < t_min) { // Find the first intersection along the segment
                t_min = t;
                intersection_point = candidate;
            }
        }
    }

    if(!intersection_point.size()) {
        throw std::runtime_error("No valid intersection found!\n");
    }

    return intersection_point;
}


template <typename T, int Dim>
inline void pathCover(const std::vector<Eigen::Matrix<T, Dim, 1>> &obstacle_pts, 
                      const std::vector<Eigen::Matrix<T, Dim, 1>> &path,
                      const Eigen::Matrix<T, -1, Dim> &A_bound, 
                      const Eigen::Matrix<T, -1, 1> &b_bound,
                      std::vector<Eigen::Matrix<T, -1, Dim>> &A, 
                      std::vector<Eigen::Matrix<T, -1, 1>> &b,
                      std::vector<Eigen::Matrix<T, Dim, 1>> &seed, 
                      std::vector<std::vector<int>> &all_pts_remaining,
                      double offset = 0,
                      int max_horizon = 1,
                      T alpha = T(0.01)) {

    Eigen::Matrix<T, -1, Dim> A_;
    Eigen::Matrix<T, -1, 1> b_;
    Eigen::Matrix<T, Dim, 1> intersection = path[0]; // Initialize to prevent garbage data
    std::vector<int> pts_remaining;

    RISP<T, Dim>(obstacle_pts, path[0], A_bound, b_bound, A_, b_, pts_remaining, offset, alpha);
    A.push_back(A_);
    b.push_back(b_);
    seed.push_back(path[0]);
    all_pts_remaining.push_back(pts_remaining);

    int i = 1;
    while((i < path.size() || ((A_ * path.back()).array() > b_.array()).all())) {
        if (((A_ * path[i]).array() <= b_.array()).all()) {
            intersection = path[i]; // The point is already inside the polyhedron defined by previous constraints
            i++;
            continue;
        } 
        else {
            intersection = computePolyhedraIntersection<T, Dim>(A_, b_, intersection, path[i]);
            RISP<T, Dim>(obstacle_pts, intersection, A_bound, b_bound, A_, b_, pts_remaining, offset, alpha);
            A.push_back(A_);
            b.push_back(b_);
            seed.push_back(intersection);
            all_pts_remaining.push_back(pts_remaining);
        }

        // Stop if the maximum number of polyhedra has been reached
        if (A.size() > max_horizon) {
            break;
        }
    }   

    if (((A_ * path.back()).array() > b_.array()).all()) {
        seed.push_back(path.back());
    }
    else {
        seed.push_back(intersection);
    }
}

}

#endif // PATHCOVER_HPP