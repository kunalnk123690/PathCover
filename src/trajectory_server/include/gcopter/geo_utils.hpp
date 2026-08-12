/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "quickhull.hpp"
#include "sdlp.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <set>
#include <vector>
#include <chrono>

namespace geo_utils
{

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    inline bool findInterior(const Eigen::MatrixX4d &hPoly,
                             Eigen::Vector3d &interior)
    {
        const int m = hPoly.rows();

        Eigen::MatrixX4d A(m, 4);
        Eigen::VectorXd b(m);
        Eigen::Vector4d c, x;
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        A.rightCols<1>().setConstant(1.0);
        b = -hPoly.rightCols<1>().array() / hNorm;
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
        interior = x.head<3>();

        return minmaxsd < 0.0 && !std::isinf(minmaxsd);
    }

    inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                        const Eigen::MatrixX4d &hPoly1,
                        const double eps = 1.0e-6)

    {
        const int m = hPoly0.rows();
        const int n = hPoly1.rows();
        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);
        A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();
        A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
        A.rightCols<1>().setConstant(1.0);
        b.topRows(m) = -hPoly0.rightCols<1>();
        b.bottomRows(n) = -hPoly1.rightCols<1>();
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

    struct filterLess
    {
        inline bool operator()(const Eigen::Vector3d &l,
                               const Eigen::Vector3d &r) const
        {
            return l(0) < r(0) ||
                   (l(0) == r(0) &&
                    (l(1) < r(1) ||
                     (l(1) == r(1) &&
                      l(2) < r(2))));
        }
    };

    inline void filterVs(const Eigen::Matrix3Xd &rV,
                         const double &epsilon,
                         Eigen::Matrix3Xd &fV)
    {
        const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
        const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
        std::set<Eigen::Vector3d, filterLess> filter;
        fV = rV;
        int offset = 0;
        Eigen::Vector3d quanti;
        for (int i = 0; i < rV.cols(); i++)
        {
            quanti = (rV.col(i) / res).array().round();
            if (filter.find(quanti) == filter.end())
            {
                filter.insert(quanti);
                fV.col(offset) = rV.col(i);
                offset++;
            }
        }
        fV = fV.leftCols(offset).eval();
        return;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    inline void enumerateVs(const Eigen::MatrixX4d &hPoly,
                            const Eigen::Vector3d &inner,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<3>() * inner;
        const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<3>().array().colwise() / b.array()).transpose();

        quickhull::QuickHull<double> qh;
        const double qhullEps = std::min(epsilon, quickhull::defaultEps<double>());
        // CCW is false because the normal in quickhull towards interior
        const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);
        const auto &idBuffer = cvxHull.getIndexBuffer();
        const int hNum = idBuffer.size() / 3;
        Eigen::Matrix3Xd rV(3, hNum);
        Eigen::Vector3d normal, point, edge0, edge1;
        for (int i = 0; i < hNum; i++)
        {
            point = A.col(idBuffer[3 * i + 1]);
            edge0 = point - A.col(idBuffer[3 * i]);
            edge1 = A.col(idBuffer[3 * i + 2]) - point;
            normal = edge0.cross(edge1); //cross in CW gives an outter normal
            rV.col(i) = normal / normal.dot(point);
        }
        filterVs(rV, epsilon, vPoly);
        vPoly = (vPoly.array().colwise() + inner.array()).eval();
        return;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    inline bool enumerateVs(const Eigen::MatrixX4d &hPoly,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        Eigen::Vector3d inner;
        if (findInterior(hPoly, inner))
        {
            enumerateVs(hPoly, inner, vPoly, epsilon);
            return true;
        }
        else
        {
            return false;
        }
    }

    // ------------------------------------------------------------------
    // Planar (2D) counterparts, added for the Jackal port.
    //
    // Same polar-duality trick as the 3D versions above: translate the
    // polytope so a strict interior point sits at the origin, map each
    // half-space to a dual point a_i/b_i, take the convex hull of those
    // points, and read one primal vertex off each dual *facet*. In 2D a
    // facet is just a hull edge, so the 3D code's quickhull + cross-product
    // normal becomes a monotone-chain hull + perpendicular -- no quickhull
    // (which only builds 3D hulls) is involved.
    //
    // Each row of hPoly is h0, h1, h2 meaning h0*x + h1*y + h2 <= 0.
    // ------------------------------------------------------------------

    inline bool findInterior2d(const Eigen::MatrixX3d &hPoly,
                               Eigen::Vector2d &interior)
    {
        const int m = hPoly.rows();

        Eigen::MatrixX3d A(m, 3);
        Eigen::VectorXd b(m);
        Eigen::Vector3d c, x;
        const Eigen::ArrayXd hNorm = hPoly.leftCols<2>().rowwise().norm();
        A.leftCols<2>() = hPoly.leftCols<2>().array().colwise() / hNorm;
        A.rightCols<1>().setConstant(1.0);
        b = -hPoly.rightCols<1>().array() / hNorm;
        c.setZero();
        c(2) = -1.0;

        const double minmaxsd = sdlp::linprog<3>(c, A, b, x);
        interior = x.head<2>();

        return minmaxsd < 0.0 && !std::isinf(minmaxsd);
    }

    struct filterLess2d
    {
        inline bool operator()(const Eigen::Vector2d &l,
                               const Eigen::Vector2d &r) const
        {
            return l(0) < r(0) || (l(0) == r(0) && l(1) < r(1));
        }
    };

    inline void filterVs2d(const Eigen::Matrix2Xd &rV,
                           const double &epsilon,
                           Eigen::Matrix2Xd &fV)
    {
        const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
        const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
        std::set<Eigen::Vector2d, filterLess2d> filter;
        fV = rV;
        int offset = 0;
        Eigen::Vector2d quanti;
        for (int i = 0; i < rV.cols(); i++)
        {
            quanti = (rV.col(i) / res).array().round();
            if (filter.find(quanti) == filter.end())
            {
                filter.insert(quanti);
                fV.col(offset) = rV.col(i);
                offset++;
            }
        }
        fV = fV.leftCols(offset).eval();
        return;
    }

    // Andrew's monotone chain. Returns hull point indices in CCW order.
    inline void convexHull2d(const Eigen::Matrix2Xd &pts,
                             std::vector<int> &hull)
    {
        hull.clear();
        const int n = pts.cols();
        if (n < 3)
        {
            for (int i = 0; i < n; i++)
            {
                hull.push_back(i);
            }
            return;
        }

        std::vector<int> order(n);
        for (int i = 0; i < n; i++)
        {
            order[i] = i;
        }
        std::sort(order.begin(), order.end(),
                  [&pts](int a, int b) {
                      return pts(0, a) < pts(0, b) ||
                             (pts(0, a) == pts(0, b) && pts(1, a) < pts(1, b));
                  });

        const auto cross = [&pts](int o, int a, int b) {
            return (pts(0, a) - pts(0, o)) * (pts(1, b) - pts(1, o)) -
                   (pts(1, a) - pts(1, o)) * (pts(0, b) - pts(0, o));
        };

        std::vector<int> chain(2 * n);
        int k = 0;
        for (int i = 0; i < n; i++)
        {
            while (k >= 2 && cross(chain[k - 2], chain[k - 1], order[i]) <= 0.0)
            {
                k--;
            }
            chain[k++] = order[i];
        }
        for (int i = n - 2, t = k + 1; i >= 0; i--)
        {
            while (k >= t && cross(chain[k - 2], chain[k - 1], order[i]) <= 0.0)
            {
                k--;
            }
            chain[k++] = order[i];
        }
        // The last point repeats the first one.
        chain.resize(k - 1);
        hull = chain;
        return;
    }

    inline void enumerateVs2d(const Eigen::MatrixX3d &hPoly,
                              const Eigen::Vector2d &inner,
                              Eigen::Matrix2Xd &vPoly,
                              const double epsilon = 1.0e-6)
    {
        const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<2>() * inner;
        const Eigen::Matrix<double, 2, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<2>().array().colwise() / b.array()).transpose();

        std::vector<int> hull;
        convexHull2d(A, hull);

        const int hNum = hull.size();
        Eigen::Matrix2Xd rV(2, hNum);
        int cnt = 0;
        for (int i = 0; i < hNum; i++)
        {
            // Dual facet = hull edge (point -> next). Its normal maps back to
            // one vertex of the primal polytope.
            const Eigen::Vector2d point = A.col(hull[i]);
            const Eigen::Vector2d next = A.col(hull[(i + 1) % hNum]);
            const Eigen::Vector2d edge = next - point;
            const Eigen::Vector2d normal(edge(1), -edge(0));
            const double denom = normal.dot(point);
            if (fabs(denom) < DBL_EPSILON)
            {
                // Edge through the origin: unbounded direction, no finite vertex.
                continue;
            }
            rV.col(cnt++) = normal / denom;
        }
        rV = rV.leftCols(cnt).eval();

        filterVs2d(rV, epsilon, vPoly);
        vPoly = (vPoly.array().colwise() + inner.array()).eval();
        return;
    }

    inline bool enumerateVs2d(const Eigen::MatrixX3d &hPoly,
                              Eigen::Matrix2Xd &vPoly,
                              const double epsilon = 1.0e-6)
    {
        Eigen::Vector2d inner;
        if (findInterior2d(hPoly, inner))
        {
            enumerateVs2d(hPoly, inner, vPoly, epsilon);
            // A planar polytope needs at least a triangle to be a usable
            // V-representation; anything less means the H-description was
            // degenerate or unbounded.
            return vPoly.cols() >= 3;
        }
        else
        {
            return false;
        }
    }

} // namespace geo_utils

#endif
