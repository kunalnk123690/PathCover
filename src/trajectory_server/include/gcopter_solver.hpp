#ifndef PATHCOVER_SELECTED_GCOPTER_SOLVER_HPP
#define PATHCOVER_SELECTED_GCOPTER_SOLVER_HPP

#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

#if PATHCOVER_DIM == 2
#ifndef TRAJECTORY_SERVER_GCOPTER_SOLVER_HPP
#define TRAJECTORY_SERVER_GCOPTER_SOLVER_HPP

// Planar (2D) specialization of gcopter::GCOPTER_PolytopeSFC (see
// gcopter/gcopter.hpp) for a differential-drive ground robot, supporting
// either:
//   S = 3 : MINCO_S3NU<2>, degree-5 pieces, minimum-JERK cost   (boundary = P,V,A)
//   S = 4 : MINCO_S4NU<2>, degree-7 pieces, minimum-SNAP cost   (boundary = P,V,A,J)
//
// Two things differ from the original quadrotor version:
//
//   1. Everything is 2D. Corridor polytopes are half-planes (each row of an
//      hPoly is h0*x + h1*y + h2 <= 0), V-polytopes are 2xN, and the MINCO /
//      Trajectory templates are instantiated at Dim = 2. Vertex enumeration
//      goes through geo_utils::enumerateVs2d rather than the quickhull-backed
//      3D path.
//
//   2. The flatness map is gone. A quadrotor's differential flatness turns
//      (vel, acc, jer) into thrust / attitude / body rates, and the original
//      penalty functional bounded those. None of them mean anything for a
//      Jackal, so attachPenaltyFunctional instead bounds what actually limits
//      a diff-drive robot: speed, acceleration, and the yaw rate implied by
//      the path's curvature (omega = (v x a) / |v|^2). The curvature term is
//      what keeps the trajectory nonholonomically trackable -- without it the
//      optimizer is free to produce corners the robot cannot turn through at
//      speed. All three have closed-form gradients w.r.t. vel/acc, so the
//      flatness backward pass disappears entirely.

#include "gcopter/geo_utils.hpp"
#include "gcopter/minco.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace trajectory_server
{

    template <int S>
    struct MincoTraits;

    template <>
    struct MincoTraits<3>
    {
        using Minco = minco::MINCO_S3NU<2>;
        using Boundary = Eigen::Matrix<double, 2, 3>; // columns: P, V, A
        static constexpr int Degree = 5;
    };

    template <>
    struct MincoTraits<4>
    {
        using Minco = minco::MINCO_S4NU<2>;
        using Boundary = Eigen::Matrix<double, 2, 4>; // columns: P, V, A, J
        static constexpr int Degree = 7;
    };

    template <int S>
    class GcopterSolver
    {
    public:
        using Traits = MincoTraits<S>;
        using BoundaryT = typename Traits::Boundary;
        static constexpr int Degree = Traits::Degree;
        static constexpr int CoeffRows = Degree + 1;

        typedef Eigen::Matrix2Xd PolyhedronV;
        typedef Eigen::MatrixX3d PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;
        typedef Eigen::Matrix<double, Eigen::Dynamic, 2> CoeffMat;

    private:
        typename Traits::Minco minco;

        double rho;
        BoundaryT headState;
        BoundaryT tailState;

        PolyhedraV vPolytopes;
        PolyhedraH hPolytopes;
        // hPolytopes shrunk inward by the safety margin. The corridor plays two
        // independent roles and they want different geometry:
        //
        //   hPolytopes       -> processCorridor() -> vPolytopes, which is what
        //                       PARAMETERIZES the waypoints. This one must keep
        //                       its overlaps: shrinking it is what made setup()
        //                       reject corridors that were perfectly fine.
        //   penaltyPolytopes -> attachPenaltyFunctional(), the SOFT containment
        //                       penalty. This is the only thing that creates
        //                       standoff, because smoothedL1() is exactly zero
        //                       anywhere inside the polytope -- so with no
        //                       margin the trajectory pays nothing for riding
        //                       the boundary and time-minimization pushes it
        //                       there.
        //
        // Shrinking only the penalty copy gets the clearance without ever
        // costing an overlap. The offset is additionally capped, per polytope,
        // by buildPenaltyPolytopes() -- an over-shrunk penalty polytope is NOT
        // free. Once it has no interior the containment term is unsatisfiable
        // at every sample point, which leaves an irreducible cost floor at the
        // full position weight competing with the time and dynamics terms, and
        // near a handoff it fights the waypoint that the (unshrunk) overlap
        // parameterization is holding there. Capping keeps both the penalty
        // polytopes and their consecutive intersections non-empty. It is also
        // strictly safer than the old scheme -- the penalty activates earlier
        // than it would on the raw corridor, so containment in the true
        // corridor is enforced at least as hard.
        PolyhedraH penaltyPolytopes;
        // Inward offset actually applied to each penalty polytope: the
        // requested margin wherever the corridor could afford it, less where
        // buildPenaltyPolytopes() had to cap it. Kept for diagnostics.
        Eigen::VectorXd appliedMargins;
        Eigen::Matrix2Xd shortPath;

        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;
        Eigen::VectorXi hPolyIdx;

        int polyN;
        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;
        double curvatureEps;
        double allocSpeed;

        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix2Xd points;
        Eigen::VectorXd times;
        Eigen::Matrix2Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        CoeffMat partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;

    private:
        static inline void forwardT(const Eigen::VectorXd &tau,
                                     Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                      EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                          const Eigen::VectorXd &gradT,
                                          EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        static inline void forwardP(const Eigen::VectorXd &xi,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     Eigen::Matrix2Xd &P)
        {
            const int sizeP = vIdx.size();
            P.resize(2, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix2Xd &ovPoly = *(Eigen::Matrix2Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector2d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                           ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                  r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix2Xd &P,
                                      const Eigen::VectorXi &vIdx,
                                      const PolyhedraV &vPolys,
                                      EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix2Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();

                ovPoly.resize(2, k + 1);
                ovPoly.col(0) = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(x,
                                       minSqrD,
                                       &GcopterSolver::costTinyNLS,
                                       nullptr,
                                       nullptr,
                                       &ovPoly,
                                       tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                          const Eigen::VectorXi &vIdx,
                                          const PolyhedraV &vPolys,
                                          const Eigen::Matrix2Xd &gradP,
                                          EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                     unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                                const Eigen::VectorXi &vIdx,
                                                const PolyhedraV &vPolys,
                                                double &cost,
                                                EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q = xi.segment(j, k);
                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        static inline bool smoothedL1(const double &x,
                                       const double &mu,
                                       double &f,
                                       double &df)
        {
            if (x < 0.0)
            {
                return false;
            }
            else if (x > mu)
            {
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        // Generic derivative basis for a degree-`Degree` monomial piece,
        // evaluated at local time s. beta_k(idx) is the coefficient of the
        // idx-th monomial coefficient contributing to the k-th time derivative:
        // beta_k(idx) = falling_factorial(idx, k) * s^(idx-k) for idx >= k, else
        // 0. This generalizes the original gcopter.hpp's hardcoded 6-term
        // (degree-5) unrolling to any degree.
        static inline void fillDerivativeBasis(double s,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta0,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta1,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta2,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta3)
        {
            beta0.setZero();
            beta1.setZero();
            beta2.setZero();
            beta3.setZero();

            double sPow[CoeffRows];
            sPow[0] = 1.0;
            for (int p = 1; p < CoeffRows; p++)
            {
                sPow[p] = sPow[p - 1] * s;
            }

            for (int idx = 0; idx <= Degree; idx++)
            {
                beta0(idx) = sPow[idx];
                if (idx >= 1)
                {
                    beta1(idx) = idx * sPow[idx - 1];
                }
                if (idx >= 2)
                {
                    beta2(idx) = idx * (idx - 1) * sPow[idx - 2];
                }
                if (idx >= 3)
                {
                    beta3(idx) = idx * (idx - 1) * (idx - 2) * sPow[idx - 3];
                }
            }
        }

        // magnitudeBounds = [v_max, a_max, omega_max]
        // penaltyWeights  = [pos_weight, vel_weight, acc_weight, omega_weight]
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                    const CoeffMat &coeffs,
                                                    const Eigen::VectorXi &hIdx,
                                                    const PolyhedraH &hPolys,
                                                    const double &smoothFactor,
                                                    const int &integralResolution,
                                                    const Eigen::VectorXd &magnitudeBounds,
                                                    const Eigen::VectorXd &penaltyWeights,
                                                    const double &curvEps,
                                                    double &cost,
                                                    Eigen::VectorXd &gradT,
                                                    CoeffMat &gradC)
        {
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double accSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double omgSqrMax = magnitudeBounds(2) * magnitudeBounds(2);

            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            const double weightAcc = penaltyWeights(2);
            const double weightOmg = penaltyWeights(3);

            Eigen::Vector2d pos, vel, acc, jer;
            Eigen::Vector2d gradPos, gradVel, gradAcc;

            double step, alpha;
            Eigen::Matrix<double, CoeffRows, 1> beta0, beta1, beta2, beta3;
            Eigen::Vector2d outerNormal;
            int K, L;
            double violaPos, violaVel, violaAcc, violaOmg;
            double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaOmgPenaD;
            double violaPosPena, violaVelPena, violaAccPena, violaOmgPena;
            double node, pena;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, CoeffRows, 2> &c = coeffs.template block<CoeffRows, 2>(i * CoeffRows, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {
                    fillDerivativeBasis(j * step, beta0, beta1, beta2, beta3);
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;

                    gradPos.setZero(), gradVel.setZero(), gradAcc.setZero();
                    pena = 0.0;

                    // --- Corridor containment -------------------------------
                    L = hIdx(i);
                    K = hPolys[L].rows();
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].template block<1, 2>(k, 0);
                        violaPos = outerNormal.dot(pos) + hPolys[L](k, 2);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }

                    // --- Speed limit ----------------------------------------
                    violaVel = vel.squaredNorm() - velSqrMax;
                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                        pena += weightVel * violaVelPena;
                    }

                    // --- Acceleration limit ---------------------------------
                    violaAcc = acc.squaredNorm() - accSqrMax;
                    if (smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD))
                    {
                        gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                        pena += weightAcc * violaAccPena;
                    }

                    // --- Yaw-rate (curvature) limit -------------------------
                    // omega = (vx*ay - vy*ax) / |v|^2, so
                    //     omega^2 - omega_max^2 = cr^2 / (|v|^4 + eps) - omega_max^2.
                    // The eps in the denominator is what keeps this finite as
                    // the robot slows to a stop (where the yaw rate of the
                    // geometric path genuinely diverges but the robot is free
                    // to just spin in place).
                    //
                    // The violation is deliberately left as this ratio rather
                    // than being multiplied out to the polynomial
                    // cr^2 - omega_max^2 (|v|^4 + eps). Multiplying through
                    // describes the same feasible set with much better
                    // conditioning (no 1/(|v|^4+eps)^2 in the gradient), but it
                    // also scales the violation by (|v|^4 + eps), which
                    // under-weights exactly the slow tight turns the limit
                    // exists to prevent: measured against a binding
                    // omg_max = 0.5 rad/s the multiplied form let the solved
                    // path reach 2.1 rad/s, where this form held it to 0.50.
                    // Keeping the violation in true omega^2 units is what makes
                    // the penalty weight mean the same thing at every speed.
                    {
                        const double crossVA = vel(0) * acc(1) - vel(1) * acc(0);
                        const double sqrSpeed = vel.squaredNorm();
                        const double den = sqrSpeed * sqrSpeed + curvEps;
                        const double invDen = 1.0 / den;
                        violaOmg = crossVA * crossVA * invDen - omgSqrMax;
                        if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                        {
                            const double w = weightOmg * violaOmgPenaD;
                            const double twoCrossOverDen = 2.0 * crossVA * invDen;
                            const double crossSqrOverDenSqr = crossVA * crossVA * invDen * invDen;

                            // d(omega^2)/d(vel): chain rule through both the
                            // cross product and the |v|^4 in the denominator.
                            gradVel(0) += w * (twoCrossOverDen * acc(1) -
                                               crossSqrOverDenSqr * 4.0 * sqrSpeed * vel(0));
                            gradVel(1) += w * (-twoCrossOverDen * acc(0) -
                                               crossSqrOverDenSqr * 4.0 * sqrSpeed * vel(1));

                            // d(omega^2)/d(acc): only through the cross product.
                            gradAcc(0) += w * (-twoCrossOverDen * vel(1));
                            gradAcc(1) += w * (twoCrossOverDen * vel(0));

                            pena += weightOmg * violaOmgPena;
                        }
                    }

                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;
                    gradC.template block<CoeffRows, 2>(i * CoeffRows, 0) += (beta0 * gradPos.transpose() +
                                                                             beta1 * gradVel.transpose() +
                                                                             beta2 * gradAcc.transpose()) *
                                                                            node * step;
                    gradT(i) += (gradPos.dot(vel) +
                                 gradVel.dot(acc) +
                                 gradAcc.dot(jer)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    cost += node * step * pena;
                }
            }

            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GcopterSolver &obj = *(GcopterSolver *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

            forwardT(tau, obj.times);
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

            double cost;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.penaltyPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.curvatureEps,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);

            cost += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector2d &ini = *((const Eigen::Vector2d *)(dataPtrs[1]));
            const Eigen::Vector2d &fin = *((const Eigen::Vector2d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix2Xd gradP = Eigen::Matrix2Xd::Zero(2, overlaps);
            Eigen::Vector2d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        static inline void getShortestPath(const Eigen::Vector2d &ini,
                                           const Eigen::Vector2d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix2Xd &path)
        {
            const int overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GcopterSolver::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(2, overlaps + 2);
            path.leftCols<1>() = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        // Fraction of a polytope's inscribed radius that the safety margin is
        // allowed to consume. The complement is what survives as interior, so
        // this is the single knob trading standoff against the guarantee below;
        // anything strictly less than 1 preserves it.
        static constexpr double marginRetention = 0.8;

        // Builds the shrunk penalty copy of the corridor, capping the inward
        // offset per polytope so the shrink can never collapse a polytope or,
        // just as important, one of the consecutive overlaps.
        //
        // Every row is unit-normal by the time this runs, so adding t to every
        // offset yields the inner parallel body at distance t, whose Chebyshev
        // radius is EXACTLY r - t. That identity makes the safe cap
        // closed-form rather than a search: t <= marginRetention * r leaves
        // (1 - marginRetention) * r of interior, positive whenever the raw
        // polytope had any.
        //
        // The same identity governs each consecutive intersection, which
        // shrinks by at most max(t_i, t_{i+1}) once both sides are offset. So
        // capping BOTH ends against that intersection's own radius is what
        // keeps the corridor sequentially overlapping after the shrink -- the
        // property processCorridor()'s decomposition and the waypoint
        // parameterization both rest on, and the one a uniform margin was free
        // to destroy in exactly the narrow passages where clearance matters
        // most. The caps only ever lower t, so their order does not matter:
        // each ends at min(margin, its own radius cap, the caps of the
        // overlaps on either side).
        //
        // Cost is 2 * polyN - 1 Seidel LPs in 3 variables per setup(),
        // negligible beside the L-BFGS solve that follows.
        static inline void buildPenaltyPolytopes(const PolyhedraH &hPs,
                                                 const double &safetyMargin,
                                                 PolyhedraH &penaltyPs,
                                                 Eigen::VectorXd &margins)
        {
            const int n = static_cast<int>(hPs.size());
            penaltyPs = hPs;
            margins.setConstant(n, 0.0);
            if (n == 0 || !(safetyMargin > 0.0))
            {
                return;
            }

            // A degenerate polytope reports a non-positive (or infinite)
            // radius; max() then leaves its margin at zero rather than
            // propagating the garbage into the offsets.
            Eigen::Vector2d centre;
            for (int i = 0; i < n; i++)
            {
                const double radius = geo_utils::inscribedRadius2d(hPs[i], centre);
                margins(i) = std::min(safetyMargin,
                                      std::max(0.0, marginRetention * radius));
            }

            for (int i = 0; i + 1 < n; i++)
            {
                const double overlapRadius =
                    geo_utils::overlapRadius2d(hPs[i], hPs[i + 1], centre);
                const double cap = std::max(0.0, marginRetention * overlapRadius);
                margins(i) = std::min(margins(i), cap);
                margins(i + 1) = std::min(margins(i + 1), cap);
            }

            for (int i = 0; i < n; i++)
            {
                penaltyPs[i].rightCols<1>().array() += margins(i);
            }
        }

        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                if (!geo_utils::enumerateVs2d(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(2, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 3);
                curIH.topRows(hPs[i].rows()) = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs2d(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(2, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs2d(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(2, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        static inline void setInitial(const Eigen::Matrix2Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix2Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            innerPoints.resize(2, sizeN - 1);
            timeAlloc.resize(sizeN);

            Eigen::Vector2d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);
                b = path.col(i + 1);
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

        // Projects a boundary state's velocity and acceleration onto what the
        // base can actually drive, mirroring the quadrotor branch.
        //
        // This matters because the initial state is MEASURED off the robot: a
        // wheel slip, a controller overshoot or a differentiation spike can
        // hand us a boundary condition outside the envelope the penalty
        // functional enforces everywhere else. Unlike those penalties the
        // boundary condition is hard -- MINCO interpolates through it exactly
        // -- so an infeasible one poisons the entire solve instead of merely
        // costing a penalty term.
        //
        // A differential-drive base has no thrust envelope or tilt to reason
        // about, so unlike the quadrotor's version this is a plain magnitude
        // clamp on each of the two bounded quantities: speed to v_max and
        // acceleration to a_max.
        inline void clampBoundaryState(BoundaryT &state) const
        {
            const double vMax = magnitudeBd(0);
            const double vNorm = state.col(1).norm();
            if (vNorm > vMax)
            {
                state.col(1) *= vMax / vNorm;
            }

            const double aMax = magnitudeBd(1);
            const double aNorm = state.col(2).norm();
            if (aNorm > aMax)
            {
                state.col(2) *= aMax / aNorm;
            }
        }

    public:
        // magnitudeBounds = [v_max, a_max, omega_max]^T
        // penaltyWeights  = [pos_weight, vel_weight, acc_weight, omega_weight]^T
        // curvatureSmoothEps regularizes the omega = (v x a)/|v|^2 denominator
        // so the yaw-rate penalty stays finite at a standstill.
        inline bool setup(const double &timeWeight,
                         const BoundaryT &initialState,
                         const BoundaryT &terminalState,
                         const PolyhedraH &safeCorridor,
                         const double &safetyMargin,
                         const double &lengthPerPiece,
                         const int &piecesPerPolytope,
                         const double &smoothingFactor,
                         const int &integralResolution,
                         const Eigen::VectorXd &magnitudeBounds,
                         const Eigen::VectorXd &penaltyWeights,
                         const double &curvatureSmoothEps)
        {
            rho = timeWeight;
            headState = initialState;
            tailState = terminalState;

            hPolytopes = safeCorridor;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<2>().rowwise().norm();
                hPolytopes[i].array().colwise() /= norms;
            }
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            // After the normalization above every row has unit normal, so
            // shifting an offset by t moves that half-space inward by exactly
            // t metres -- the identity buildPenaltyPolytopes() relies on to cap
            // the shrink. Built from the already-normalized hPolytopes and only
            // after processCorridor() has done its work, so the margin cannot
            // influence the decomposition.
            buildPenaltyPolytopes(hPolytopes, safetyMargin,
                                  penaltyPolytopes, appliedMargins);

            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            curvatureEps = curvatureSmoothEps;
            allocSpeed = magnitudeBd(0) * 3.0;

            // Only now is magnitudeBd populated, so this is the earliest the
            // boundary states can be clamped. Position is untouched, so
            // getShortestPath below is unaffected.
            clampBoundaryState(headState);
            clampBoundaryState(tailState);

            getShortestPath(headState.col(0), tailState.col(0),
                            vPolytopes, smoothEps, shortPath);
            if (piecesPerPolytope > 0)
            {
                // Fixed budget per polytope. With 1, every interior waypoint
                // lands in an overlap region between consecutive polytopes
                // (see the vPolyIdx assignment below), which is the tightest
                // parameterization the corridor admits: the trajectory has just
                // enough freedom to thread the overlaps in order and none left
                // over to wander inside a polytope and tie a knot.
                pieceIdx.setConstant(polyN, piecesPerPolytope);
            }
            else
            {
                // Adaptive: as many pieces as the shortest path is long.
                const Eigen::Matrix2Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
                pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
                pieceIdx.array() += 1;
            }
            pieceN = pieceIdx.sum();

            temporalDim = pieceN;
            spatialDim = 0;
            vPolyIdx.resize(pieceN - 1);
            hPolyIdx.resize(pieceN);
            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {
                    if (l < k - 1)
                    {
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1)
                    {
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    hPolyIdx(j) = i;
                }
            }

            minco.setConditions(headState, tailState, pieceN);

            points.resize(2, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(2, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize(CoeffRows * pieceN, 2);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        // Inward offset actually applied to each polytope's penalty copy, in
        // corridor order. Equal to the requested safety margin wherever the
        // corridor was wide enough to give it up, smaller where the polytope
        // or one of its overlaps was too narrow. Valid after setup() returns
        // true; empty before that.
        inline const Eigen::VectorXd &getAppliedMargins() const
        {
            return appliedMargins;
        }

        // maxIterations bounds the solve so one bad corridor cannot stall a
        // receding-horizon replan; 0 leaves L-BFGS uncapped. Hitting the cap
        // returns LBFGSERR_MAXIMUMITERATION (negative), so the partial iterate
        // is discarded and the caller keeps its previous trajectory.
        inline double optimize(Trajectory<Degree, 2> &traj,
                               const double &relCostTol,
                               const int &maxIterations)
        {
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            lbfgs_params.max_iterations = maxIterations;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GcopterSolver::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                         << lbfgs::lbfgs_strerror(ret)
                         << std::endl;
            }

            return minCostFunctional;
        }
    };

} // namespace trajectory_server

#endif // TRAJECTORY_SERVER_GCOPTER_SOLVER_HPP

#elif PATHCOVER_DIM == 3
#ifndef TRAJECTORY_SERVER_GCOPTER_SOLVER_HPP
#define TRAJECTORY_SERVER_GCOPTER_SOLVER_HPP

// Generalization of gcopter::GCOPTER_PolytopeSFC (see gcopter/gcopter.hpp)
// to support either:
//   S = 3 : MINCO_S3NU, degree-5 pieces, minimum-JERK cost   (boundary = P,V,A)
//   S = 4 : MINCO_S4NU, degree-7 pieces, minimum-SNAP cost   (boundary = P,V,A,J)
//
// The corridor/V-polytope machinery (forwardP/backwardP/processCorridor/
// getShortestPath/...) is degree-independent and copied verbatim from
// gcopter.hpp. The only genuinely degree-dependent piece is
// attachPenaltyFunctional's per-piece coefficient block size and Bernstein-
// like derivative basis, which is generalized below via fillDerivativeBasis
// instead of the original's hardcoded 6-term unrolling.

#include "gcopter/geo_utils.hpp"
#include "gcopter/minco.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace trajectory_server
{

    template <int S>
    struct MincoTraits;

    template <>
    struct MincoTraits<3>
    {
        using Minco = minco::MINCO_S3NU<>;
        using Boundary = Eigen::Matrix3d; // columns: P, V, A
        static constexpr int Degree = 5;
    };

    template <>
    struct MincoTraits<4>
    {
        using Minco = minco::MINCO_S4NU<>;
        using Boundary = Eigen::Matrix<double, 3, 4>; // columns: P, V, A, J
        static constexpr int Degree = 7;
    };

    template <int S>
    class GcopterSolver
    {
    public:
        using Traits = MincoTraits<S>;
        using BoundaryT = typename Traits::Boundary;
        static constexpr int Degree = Traits::Degree;
        static constexpr int CoeffRows = Degree + 1;

        typedef Eigen::Matrix3Xd PolyhedronV;
        typedef Eigen::MatrixX4d PolyhedronH;
        typedef std::vector<PolyhedronV> PolyhedraV;
        typedef std::vector<PolyhedronH> PolyhedraH;

    private:
        typename Traits::Minco minco;
        flatness::FlatnessMap flatmap;

        double rho;
        BoundaryT headState;
        BoundaryT tailState;

        PolyhedraV vPolytopes;
        PolyhedraH hPolytopes;
        // hPolytopes shrunk inward by the safety margin. The corridor plays two
        // independent roles and they want different geometry:
        //
        //   hPolytopes       -> processCorridor() -> vPolytopes, which is what
        //                       PARAMETERIZES the waypoints. This one must keep
        //                       its overlaps: shrinking it is what made setup()
        //                       reject corridors that were perfectly fine.
        //   penaltyPolytopes -> attachPenaltyFunctional(), the SOFT containment
        //                       penalty. This is the only thing that creates
        //                       standoff, because smoothedL1() is exactly zero
        //                       anywhere inside the polytope -- so with no
        //                       margin the trajectory pays nothing for riding
        //                       the boundary and time-minimization pushes it
        //                       there.
        //
        // Shrinking only the penalty copy gets the clearance without ever
        // costing an overlap. The offset is additionally capped, per polytope,
        // by buildPenaltyPolytopes() -- an over-shrunk penalty polytope is NOT
        // free. Once it has no interior the containment term is unsatisfiable
        // at every sample point, which leaves an irreducible cost floor at the
        // full position weight competing with the time and dynamics terms, and
        // near a handoff it fights the waypoint that the (unshrunk) overlap
        // parameterization is holding there. Capping keeps both the penalty
        // polytopes and their consecutive intersections non-empty. It is also
        // strictly safer than the old scheme -- the penalty activates earlier
        // than it would on the raw corridor, so containment in the true
        // corridor is enforced at least as hard.
        PolyhedraH penaltyPolytopes;
        // Inward offset actually applied to each penalty polytope: the
        // requested margin wherever the corridor could afford it, less where
        // buildPenaltyPolytopes() had to cap it. Kept for diagnostics.
        Eigen::VectorXd appliedMargins;
        Eigen::Matrix3Xd shortPath;

        Eigen::VectorXi pieceIdx;
        Eigen::VectorXi vPolyIdx;
        Eigen::VectorXi hPolyIdx;

        int polyN;
        int pieceN;

        int spatialDim;
        int temporalDim;

        double smoothEps;
        int integralRes;
        Eigen::VectorXd magnitudeBd;
        Eigen::VectorXd penaltyWt;
        Eigen::VectorXd physicalPm;
        double allocSpeed;

        lbfgs::lbfgs_parameter_t lbfgs_params;

        Eigen::Matrix3Xd points;
        Eigen::VectorXd times;
        Eigen::Matrix3Xd gradByPoints;
        Eigen::VectorXd gradByTimes;
        Eigen::MatrixX3d partialGradByCoeffs;
        Eigen::VectorXd partialGradByTimes;

    private:
        static inline void forwardT(const Eigen::VectorXd &tau,
                                     Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                      EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                          const Eigen::VectorXd &gradT,
                                          EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        static inline void forwardP(const Eigen::VectorXd &xi,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     Eigen::Matrix3Xd &P)
        {
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k).normalized().head(k - 1);
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        static inline double costTinyNLS(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;

            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                           ovPoly.col(1) - ovPoly.col(0);

            double cost = delta.squaredNorm();
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                  r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                      const Eigen::VectorXi &vIdx,
                                      const PolyhedraV &vPolys,
                                      EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();

                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i);
                ovPoly.rightCols(k) = vPolys[l];
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));
                lbfgs::lbfgs_optimize(x,
                                       minSqrD,
                                       &GcopterSolver::costTinyNLS,
                                       nullptr,
                                       nullptr,
                                       &ovPoly,
                                       tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                          const Eigen::VectorXi &vIdx,
                                          const PolyhedraV &vPolys,
                                          const Eigen::Matrix3Xd &gradP,
                                          EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;
                gradQ.resize(k);
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                     unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                                const Eigen::VectorXi &vIdx,
                                                const PolyhedraV &vPolys,
                                                double &cost,
                                                EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();

                q = xi.segment(j, k);
                sqrNormQ = q.squaredNorm();
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        static inline bool smoothedL1(const double &x,
                                       const double &mu,
                                       double &f,
                                       double &df)
        {
            if (x < 0.0)
            {
                return false;
            }
            else if (x > mu)
            {
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                const double xdmu = x / mu;
                const double sqrxdmu = xdmu * xdmu;
                const double mumxd2 = mu - 0.5 * x;
                f = mumxd2 * sqrxdmu * xdmu;
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        // Generic Bernstein-style derivative basis for a degree-`Degree`
        // monomial piece, evaluated at local time s. beta_k(idx) is the
        // coefficient of the idx-th monomial coefficient contributing to the
        // k-th time derivative: beta_k(idx) = falling_factorial(idx, k) *
        // s^(idx-k) for idx >= k, else 0. This generalizes the original
        // gcopter.hpp's hardcoded 6-term (degree-5) unrolling to any degree.
        static inline void fillDerivativeBasis(double s,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta0,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta1,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta2,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta3,
                                                Eigen::Matrix<double, CoeffRows, 1> &beta4)
        {
            beta0.setZero();
            beta1.setZero();
            beta2.setZero();
            beta3.setZero();
            beta4.setZero();

            double sPow[CoeffRows];
            sPow[0] = 1.0;
            for (int p = 1; p < CoeffRows; p++)
            {
                sPow[p] = sPow[p - 1] * s;
            }

            for (int idx = 0; idx <= Degree; idx++)
            {
                beta0(idx) = sPow[idx];
                if (idx >= 1)
                {
                    beta1(idx) = idx * sPow[idx - 1];
                }
                if (idx >= 2)
                {
                    beta2(idx) = idx * (idx - 1) * sPow[idx - 2];
                }
                if (idx >= 3)
                {
                    beta3(idx) = idx * (idx - 1) * (idx - 2) * sPow[idx - 3];
                }
                if (idx >= 4)
                {
                    beta4(idx) = idx * (idx - 1) * (idx - 2) * (idx - 3) * sPow[idx - 4];
                }
            }
        }

        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                    const Eigen::MatrixX3d &coeffs,
                                                    const Eigen::VectorXi &hIdx,
                                                    const PolyhedraH &hPolys,
                                                    const double &smoothFactor,
                                                    const int &integralResolution,
                                                    const Eigen::VectorXd &magnitudeBounds,
                                                    const Eigen::VectorXd &penaltyWeights,
                                                    flatness::FlatnessMap &flatMap,
                                                    double &cost,
                                                    Eigen::VectorXd &gradT,
                                                    Eigen::MatrixX3d &gradC)
        {
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);
            const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);
            const double thetaMax = magnitudeBounds(2);
            const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));
            const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));
            const double thrustSqrRadi = thrustRadi * thrustRadi;

            const double weightPos = penaltyWeights(0);
            const double weightVel = penaltyWeights(1);
            const double weightOmg = penaltyWeights(2);
            const double weightTheta = penaltyWeights(3);
            const double weightThrust = penaltyWeights(4);

            Eigen::Vector3d pos, vel, acc, jer, sna;
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;
            double thr, cos_theta;
            Eigen::Vector4d quat;
            Eigen::Vector3d omg;
            double gradThr;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradPos, gradVel, gradOmg;

            double step, alpha;
            Eigen::Matrix<double, CoeffRows, 1> beta0, beta1, beta2, beta3, beta4;
            Eigen::Vector3d outerNormal;
            int K, L;
            double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
            double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
            double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
            double node, pena;

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, CoeffRows, 3> &c = coeffs.template block<CoeffRows, 3>(i * CoeffRows, 0);
                step = T(i) * integralFrac;
                for (int j = 0; j <= integralResolution; j++)
                {
                    fillDerivativeBasis(j * step, beta0, beta1, beta2, beta3, beta4);
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;

                    flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

                    violaVel = vel.squaredNorm() - velSqrMax;
                    violaOmg = omg.squaredNorm() - omgSqrMax;
                    cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));
                    violaTheta = acos(cos_theta) - thetaMax;
                    violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                    gradThr = 0.0;
                    gradQuat.setZero();
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    pena = 0.0;

                    L = hIdx(i);
                    K = hPolys[L].rows();
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].block<1, 3>(k, 0);
                        violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }

                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                        pena += weightVel * violaVelPena;
                    }

                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                    {
                        gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                        pena += weightOmg * violaOmgPena;
                    }

                    if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
                    {
                        gradQuat += weightTheta * violaThetaPenaD /
                                    sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                                    Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                        pena += weightTheta * violaThetaPena;
                    }

                    if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))
                    {
                        gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
                        pena += weightThrust * violaThrustPena;
                    }

                    flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                                     totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                     totalGradPsi, totalGradPsiD);

                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;
                    gradC.template block<CoeffRows, 3>(i * CoeffRows, 0) += (beta0 * totalGradPos.transpose() +
                                                                              beta1 * totalGradVel.transpose() +
                                                                              beta2 * totalGradAcc.transpose() +
                                                                              beta3 * totalGradJer.transpose()) *
                                                                             node * step;
                    gradT(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * node * step +
                                node * integralFrac * pena;
                    cost += node * step * pena;
                }
            }

            return;
        }

        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GcopterSolver &obj = *(GcopterSolver *)ptr;
            const int dimTau = obj.temporalDim;
            const int dimXi = obj.spatialDim;
            const double weightT = obj.rho;
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

            forwardT(tau, obj.times);
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

            double cost;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.penaltyPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.flatmap,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);

            cost += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;

            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                a = i == 0 ? ini : b;
                if (i < overlaps)
                {
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GcopterSolver::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;
            path.rightCols<1>() = fin;
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        // Fraction of a polytope's inscribed radius that the safety margin is
        // allowed to consume. The complement is what survives as interior, so
        // this is the single knob trading standoff against the guarantee below;
        // anything strictly less than 1 preserves it.
        static constexpr double marginRetention = 0.8;

        // Builds the shrunk penalty copy of the corridor, capping the inward
        // offset per polytope so the shrink can never collapse a polytope or,
        // just as important, one of the consecutive overlaps.
        //
        // Every row is unit-normal by the time this runs, so adding t to every
        // offset yields the inner parallel body at distance t, whose Chebyshev
        // radius is EXACTLY r - t. That identity makes the safe cap
        // closed-form rather than a search: t <= marginRetention * r leaves
        // (1 - marginRetention) * r of interior, positive whenever the raw
        // polytope had any.
        //
        // The same identity governs each consecutive intersection, which
        // shrinks by at most max(t_i, t_{i+1}) once both sides are offset. So
        // capping BOTH ends against that intersection's own radius is what
        // keeps the corridor sequentially overlapping after the shrink -- the
        // property processCorridor()'s decomposition and the waypoint
        // parameterization both rest on, and the one a uniform margin was free
        // to destroy in exactly the narrow passages where clearance matters
        // most. The caps only ever lower t, so their order does not matter:
        // each ends at min(margin, its own radius cap, the caps of the
        // overlaps on either side).
        //
        // Cost is 2 * polyN - 1 Seidel LPs in 4 variables per setup(),
        // negligible beside the L-BFGS solve that follows.
        static inline void buildPenaltyPolytopes(const PolyhedraH &hPs,
                                                 const double &safetyMargin,
                                                 PolyhedraH &penaltyPs,
                                                 Eigen::VectorXd &margins)
        {
            const int n = static_cast<int>(hPs.size());
            penaltyPs = hPs;
            margins.setConstant(n, 0.0);
            if (n == 0 || !(safetyMargin > 0.0))
            {
                return;
            }

            // A degenerate polytope reports a non-positive (or infinite)
            // radius; max() then leaves its margin at zero rather than
            // propagating the garbage into the offsets.
            Eigen::Vector3d centre;
            for (int i = 0; i < n; i++)
            {
                const double radius = geo_utils::inscribedRadius(hPs[i], centre);
                margins(i) = std::min(safetyMargin,
                                      std::max(0.0, marginRetention * radius));
            }

            for (int i = 0; i + 1 < n; i++)
            {
                const double overlapRadius =
                    geo_utils::overlapRadius(hPs[i], hPs[i + 1], centre);
                const double cap = std::max(0.0, marginRetention * overlapRadius);
                margins(i) = std::min(margins(i), cap);
                margins(i + 1) = std::min(margins(i + 1), cap);
            }

            for (int i = 0; i < n; i++)
            {
                penaltyPs[i].rightCols<1>().array() += margins(i);
            }
        }

        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);

                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows()) = hPs[i];
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();
            const int sizeN = intervalNs.sum();
            innerPoints.resize(3, sizeN - 1);
            timeAlloc.resize(sizeN);

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);
                a = path.col(i);
                b = path.col(i + 1);
                c = (b - a) / l;
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

        // Projects a boundary state's velocity and acceleration onto what the
        // vehicle can actually fly, mirroring traj_opt's setBoundConds.
        //
        // This matters because the initial state is MEASURED off the vehicle: a
        // gust, a controller overshoot or a differentiation spike can hand us a
        // boundary condition outside the envelope the penalty functional
        // enforces everywhere else. Unlike those penalties the boundary
        // condition is hard -- MINCO interpolates through it exactly -- so an
        // infeasible one poisons the entire solve instead of merely costing a
        // penalty term.
        //
        // Velocity clamps to v_max. Acceleration is clamped through the same
        // quantity the flatness map actually bounds: the drag-free specific
        // thrust f = a + g*e3, whose norm is thrust/mass and whose angle from
        // vertical is the tilt. Tilt is fixed first, by shrinking the horizontal
        // component while preserving the vertical one that holds the vehicle up;
        // the norm is then scaled into [thrust_min, thrust_max]/mass. Scaling f
        // does not change its tilt, so the two steps cannot fight each other.
        inline void clampBoundaryState(BoundaryT &state) const
        {
            const double vMax = magnitudeBd(0);
            const double vNorm = state.col(1).norm();
            if (vNorm > vMax)
            {
                state.col(1) *= vMax / vNorm;
            }

            const double mass = physicalPm(0);
            const double grav = physicalPm(1);
            const double thetaMax = magnitudeBd(2);
            const double fMin = magnitudeBd(3) / mass;
            const double fMax = magnitudeBd(4) / mass;

            Eigen::Vector3d f = state.col(2);
            f(2) += grav;

            if (f(2) <= 0.0)
            {
                // Tilted at or past horizontal: no thrust vector realizes this,
                // so fall back to the minimum-thrust hover direction.
                f = Eigen::Vector3d(0.0, 0.0, fMin);
            }
            else
            {
                if (thetaMax < M_PI_2)
                {
                    const double fhMax = f(2) * std::tan(thetaMax);
                    const double fhNorm = f.head<2>().norm();
                    if (fhNorm > fhMax)
                    {
                        f.head<2>() *= fhMax / fhNorm;
                    }
                }
                // f(2) > 0 here, so fNorm > 0 and neither scaling divides by zero.
                const double fNorm = f.norm();
                if (fNorm < fMin)
                {
                    f *= fMin / fNorm;
                }
                else if (fNorm > fMax)
                {
                    f *= fMax / fNorm;
                }
            }

            f(2) -= grav;
            state.col(2) = f;
        }

    public:
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        inline bool setup(const double &timeWeight,
                         const BoundaryT &initialState,
                         const BoundaryT &terminalState,
                         const PolyhedraH &safeCorridor,
                         const double &safetyMargin,
                         const double &lengthPerPiece,
                         const int &piecesPerPolytope,
                         const double &smoothingFactor,
                         const int &integralResolution,
                         const Eigen::VectorXd &magnitudeBounds,
                         const Eigen::VectorXd &penaltyWeights,
                         const Eigen::VectorXd &physicalParams)
        {
            rho = timeWeight;
            headState = initialState;
            tailState = terminalState;

            hPolytopes = safeCorridor;
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<3>().rowwise().norm();
                hPolytopes[i].array().colwise() /= norms;
            }
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            // After the normalization above every row has unit normal, so
            // shifting an offset by t moves that half-space inward by exactly
            // t metres -- the identity buildPenaltyPolytopes() relies on to cap
            // the shrink. Built from the already-normalized hPolytopes and only
            // after processCorridor() has done its work, so the margin cannot
            // influence the decomposition.
            buildPenaltyPolytopes(hPolytopes, safetyMargin,
                                  penaltyPolytopes, appliedMargins);

            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            physicalPm = physicalParams;
            allocSpeed = magnitudeBd(0) * 3.0;

            // Only now are magnitudeBd/physicalPm populated, so this is the
            // earliest the boundary states can be clamped. Position is
            // untouched, so getShortestPath below is unaffected.
            clampBoundaryState(headState);
            clampBoundaryState(tailState);

            getShortestPath(headState.col(0), tailState.col(0),
                            vPolytopes, smoothEps, shortPath);
            if (piecesPerPolytope > 0)
            {
                // Fixed budget per polytope. With 1, every interior waypoint
                // lands in an overlap region between consecutive polytopes
                // (see the vPolyIdx assignment below), which is the tightest
                // parameterization the corridor admits: the trajectory has just
                // enough freedom to thread the overlaps in order and none left
                // over to wander inside a polytope and tie a knot.
                pieceIdx.setConstant(polyN, piecesPerPolytope);
            }
            else
            {
                // Adaptive: as many pieces as the shortest path is long.
                const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
                pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
                pieceIdx.array() += 1;
            }
            pieceN = pieceIdx.sum();

            temporalDim = pieceN;
            spatialDim = 0;
            vPolyIdx.resize(pieceN - 1);
            hPolyIdx.resize(pieceN);
            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);
                for (int l = 0; l < k; l++, j++)
                {
                    if (l < k - 1)
                    {
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1)
                    {
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    hPolyIdx(j) = i;
                }
            }

            minco.setConditions(headState, tailState, pieceN);
            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                         physicalPm(3), physicalPm(4), physicalPm(5));

            points.resize(3, pieceN - 1);
            times.resize(pieceN);
            gradByPoints.resize(3, pieceN - 1);
            gradByTimes.resize(pieceN);
            partialGradByCoeffs.resize(CoeffRows * pieceN, 3);
            partialGradByTimes.resize(pieceN);

            return true;
        }

        // Inward offset actually applied to each polytope's penalty copy, in
        // corridor order. Equal to the requested safety margin wherever the
        // corridor was wide enough to give it up, smaller where the polytope
        // or one of its overlaps was too narrow. Valid after setup() returns
        // true; empty before that.
        inline const Eigen::VectorXd &getAppliedMargins() const
        {
            return appliedMargins;
        }

        // maxIterations bounds the solve so one bad corridor cannot stall a
        // receding-horizon replan; 0 leaves L-BFGS uncapped. Hitting the cap
        // returns LBFGSERR_MAXIMUMITERATION (negative), so the partial iterate
        // is discarded and the caller keeps its previous trajectory -- the same
        // way traj_opt treats its earlyExit cancellation as a failed solve.
        inline double optimize(Trajectory<Degree> &traj,
                               const double &relCostTol,
                               const int &maxIterations)
        {
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            setInitial(shortPath, allocSpeed, pieceIdx, points, times);
            backwardT(times, tau);
            backwardP(points, vPolyIdx, vPolytopes, xi);

            double minCostFunctional;
            lbfgs_params.mem_size = 256;
            lbfgs_params.past = 3;
            lbfgs_params.min_step = 1.0e-32;
            lbfgs_params.g_epsilon = 0.0;
            lbfgs_params.delta = relCostTol;
            lbfgs_params.max_iterations = maxIterations;

            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GcopterSolver::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            if (ret >= 0)
            {
                forwardT(tau, times);
                forwardP(xi, vPolyIdx, vPolytopes, points);
                minco.setParameters(points, times);
                minco.getTrajectory(traj);
            }
            else
            {
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                         << lbfgs::lbfgs_strerror(ret)
                         << std::endl;
            }

            return minCostFunctional;
        }
    };

} // namespace traj_opt

#endif // TRAJ_OPT_GCOPTER_SOLVER_HPP
#else
#error "trajectory_server supports only PATHCOVER_DIM=2 or 3"
#endif

#endif
