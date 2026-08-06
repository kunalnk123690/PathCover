#ifndef GEOMETRIC_CONTROLLER_HPP
#define GEOMETRIC_CONTROLLER_HPP

#include <Eigen/Dense>
#include <cmath>

/**
 * @file GeometricController.hpp
 * @brief Defines the GeometricController class for quadrotor control.
 * @author Kunal Narkhede <kunalnk@udel.edu>
 */

/**
 * @class GeometricController
 * @brief Implements a geometric tracking controller for a quadrotor on SE(3).
 * @details This class provides the implementation of a nonlinear geometric tracking controller
 * based on the work by T. Lee, M. Leok, and N. H. McClamroch. The controller operates directly on the
 * Special Euclidean Group SE(3), avoiding singularities associated with Euler angles.
 *
 * The control law is divided into two main parts:
 * 1.  **Position Control**: A PD-like control law in the position space computes a desired total force vector $F$.
 * 2.  **Attitude Control**: The desired force vector determines the desired orientation $R_d$. An attitude controller then
 * generates the necessary body-frame torques $M$ to track this desired orientation.
 */
class GeometricController {
    public:
        /**
         * @brief Default constructor for the GeometricController.
         * @param mass Mass of the quadrotor.
         * @param inertia Inertia matrix of the quadrotor.
         * @param Kp Proportional gain vector for position control.
         * @param Kd Derivative gain vector for position control.
         * @param KR Proportional gain vector for attitude control.
         * @param KW Derivative gain vector for attitude control.
         * @details Initializes the controller with the provided physical parameters and gain settings.
         */
        GeometricController(const double& mass, 
                            const Eigen::Matrix3d& inertia,
                            const Eigen::Vector3d& Kp,
                            const Eigen::Vector3d& Kd,
                            const Eigen::Vector3d& KR,
                            const Eigen::Vector3d& KW) {
            m_ = mass;
            J_ = inertia;

            Kp_.diagonal() = Kp;
            Kd_.diagonal() = Kd;
            KR_.diagonal() = KR;
            KW_.diagonal() = KW;

            e1_ << 1, 0, 0;
            e2_ << 0, 1, 0;
            e3_ << 0, 0, 1;
        }


        /**
         * @brief The main controller function.
         * @details This function takes the current state and desired trajectory and computes the
         * required wrench (total thrust and body-frame torques).
         * @param[in] x Current position of the quadrotor.
         * @param[in] v Current velocity of the quadrotor.
         * @param[in] R Current rotation matrix (orientation) of the quadrotor.
         * @param[in] w Current angular velocity of the quadrotor in the body frame.
         * @param[in] xDes Desired position.
         * @param[in] vDes Desired velocity.
         * @param[in] aDes Desired acceleration (feedforward term).
         * @param[in] yaw Desired yaw angle.
         * @param[in] yaw_dot Desired yaw rate.
         * @return A 4D vector where the first element is the total thrust $f$ and the last three are the body-frame torques $M$.
         */
        Eigen::Vector4d Controller(const Eigen::Vector3d& x,
                                   const Eigen::Vector3d& v,
                                   const Eigen::Matrix3d& R,
                                   const Eigen::Vector3d& w,
                                   const Eigen::Vector3d& xDes,
                                   const Eigen::Vector3d& vDes,
                                   const Eigen::Vector3d& aDes,
                                   double yaw,
                                   double yaw_dot) {
            
            // 1. Compute Position and Velocity Errors
            Eigen::Vector3d ep = x - xDes;
            Eigen::Vector3d ev = v - vDes;

            // 2. Compute Desired Force Vector (Fd) matching the so3_control formulation
            // Fd = -Kp*ep - Kd*ev + m*g*e3 + m*aDes
            Eigen::Vector3d Fd = -Kp_.diagonal().cwiseProduct(ep) 
                                 - Kd_.diagonal().cwiseProduct(ev) 
                                 + m_ * g_ * e3_ 
                                 + m_ * aDes;

            // 3. Extract Current Body Z-axis
            Eigen::Vector3d zb = R.col(2);

            // 4. Calculate Total Thrust Scalar Projection
            double f = Fd.dot(zb);

            // 5. Construct Desired Body Coordinate Frame
            // Direction of the target Z-axis
            Eigen::Vector3d zbd = Fd.normalized();

            // Intermediate X-axis vector determined by target heading (yaw)
            Eigen::Vector3d xcd(std::cos(yaw), std::sin(yaw), 0.0);

            // Construct orthogonal Y and X axes matching standard geometry tracking
            Eigen::Vector3d ybd = zbd.cross(xcd).normalized();
            Eigen::Vector3d xbd = ybd.cross(zbd).normalized();

            Eigen::Matrix3d Rd;
            Rd.col(0) = xbd;
            Rd.col(1) = ybd;
            Rd.col(2) = zbd;

            // 6. Compute Attitude Control Error (eR)
            // Modified to reflect the direct vector tracking formulation used in so3_control
            Eigen::Vector3d eR = 0.5 * veeMap(Rd.transpose() * R - R.transpose() * Rd);

            // 7. Compute Desired Angular Velocity (wd) 
            // In so3_control, the angular feedback loops utilize direct projection errors 
            // of the feedforward heading rates combined with attitude tracking adjustments.
            Eigen::Vector3d wd;
            wd << 0.0, 0.0, yaw_dot; // Feedforward heading rate inside the frame alignment
            
            // Map the tracking velocity error vector
            Eigen::Vector3d ew = w - R.transpose() * Rd * wd;

            // 8. Compute Body-Frame Control Torques (M)
            // M = -KR*eR - KW*ew + w x (J*w)
            Eigen::Vector3d M = -KR_.diagonal().cwiseProduct(eR) 
                                - KW_.diagonal().cwiseProduct(ew) 
                                + hatMap(w) * J_ * w;

            // 9. Assemble Wrench Outputs
            Eigen::Vector4d wrench;
            wrench << f, M;
            return wrench;
        }


        /**
         * @brief The main controller function.
         * @details This function takes the current state and desired trajectory and computes the
         * required wrench (total thrust and body-frame torques).
         * @param[in] x Current position of the quadrotor.
         * @param[in] v Current velocity of the quadrotor.
         * @param[in] R Current rotation matrix (orientation) of the quadrotor.
         * @param[in] w Current angular velocity of the quadrotor in the body frame.
         * @param[in] xDes Desired position.
         * @param[in] vDes Desired velocity.
         * @param[in] aDes Desired acceleration (feedforward term).
         * @param[in] jerkDes Desired jerk (feedforward term).
         * @param[in] snapDes Desired snap (feedforward term).
         * @param[in] yaw Desired yaw angle.
         * @param[in] yaw_dot Desired yaw rate.
         * @param[in] yaw_ddot Desired yaw acceleration.
         * @return A 4D vector where the first element is the total thrust $f$ and the last three are the body-frame torques $M$.
         */
        Eigen::Vector4d Controller(const Eigen::Vector3d& x,
                                   const Eigen::Vector3d& v,
                                   const Eigen::Matrix3d& R,
                                   const Eigen::Vector3d& w,
                                   const Eigen::Vector3d& xDes,
                                   const Eigen::Vector3d& vDes,
                                   const Eigen::Vector3d& aDes,
                                   const Eigen::Vector3d& jerkDes,
                                   const Eigen::Vector3d& snapDes,
                                   double yaw,
                                   double yaw_dot,
                                   double yaw_ddot) {
            //--- Position errors ---//
            Eigen::Vector3d ep = x - xDes;
            Eigen::Vector3d ev = v - vDes;
                                        
            // Desired force Fd (world frame)
            Eigen::Vector3d Kp_diag = Kp_.diagonal();
            Eigen::Vector3d Kd_diag = Kd_.diagonal();
            Eigen::Vector3d Fd = -Kp_diag.cwiseProduct(ep) - Kd_diag.cwiseProduct(ev)
                                + m_ * g_ * e3_ + m_ * aDes;
                                        
            // Thrust magnitude
            Eigen::Vector3d zb = R.col(2);
            double f = Fd.dot(zb); // u1
                                        
            // Current acceleration estimate
            Eigen::Vector3d current_acc = -g_ * e3_ + (f * zb) / m_; // matches current_acc in MATLAB
                                        
            // accel error
            Eigen::Vector3d ea = current_acc - aDes;
                                        
            // Desired force derivative Fd_dot
            Eigen::Vector3d Fd_dot = -Kp_diag.cwiseProduct(ev) - Kd_diag.cwiseProduct(ea) + m_ * jerkDes;
                                        
            // zbd and its derivative
            double norm_Fd = Fd.norm();
            Eigen::Vector3d zbd = Fd / norm_Fd;
                                        
            Eigen::Vector3d zbd_dot = (Fd_dot * norm_Fd - Fd * (Fd.dot(Fd_dot) / norm_Fd)) / (norm_Fd * norm_Fd);
                                        
            // xcd and its derivatives from yaw
            Eigen::Vector3d xcd;
            xcd << std::cos(yaw), std::sin(yaw), 0.0;
            Eigen::Vector3d xcd_dot;
            xcd_dot << -std::sin(yaw) * yaw_dot, std::cos(yaw) * yaw_dot, 0.0;
            Eigen::Vector3d xcd_2dot;
            xcd_2dot << -std::cos(yaw) * yaw_dot * yaw_dot - std::sin(yaw) * yaw_ddot,
                        -std::sin(yaw) * yaw_dot * yaw_dot + std::cos(yaw) * yaw_ddot,
                        0.0;
                                        
            // ybd calculation
            Eigen::Vector3d hat_zbd_xcd = hatMap(zbd) * xcd;
            double norm_hat_zbd_xcd = hat_zbd_xcd.norm();
            Eigen::Vector3d ybd = (hatMap(zbd) * xcd) / norm_hat_zbd_xcd;
            // xbd
            Eigen::Vector3d xbd = hatMap(ybd) * zbd;

            // desired rotation matrix
            Eigen::Matrix3d Rd;
            Rd.col(0) = xbd;
            Rd.col(1) = ybd;
            Rd.col(2) = zbd;       
                                        
            // Orientation error eR
            Eigen::Vector3d eR = 0.5 * veeMap(Rd.transpose() * R - R.transpose() * Rd);
                                        
            // Desired angular velocity wd
            // Compute derivative of ybd
            // First compute intermediate quantities for ybd_dot
            Eigen::Vector3d hat_zbd_dot_xcd = hatMap(zbd_dot) * xcd;
            Eigen::Vector3d hat_zbd_xcd_dot = hatMap(zbd) * xcd_dot;
            Eigen::Vector3d zbd_x_xcd_dot = hat_zbd_dot_xcd + hat_zbd_xcd_dot;
                                        
            // derivative of norm(hat(zbd)*xcd)
            double zbd_xcd_norm_dot = (hat_zbd_xcd.dot(zbd_x_xcd_dot)) / norm_hat_zbd_xcd;
                                        
            // ybd_dot numerator / denominator
            Eigen::Vector3d ybd_dot_num = zbd_x_xcd_dot * norm_hat_zbd_xcd - (hatMap(zbd) * xcd) * zbd_xcd_norm_dot;
            Eigen::Vector3d ybd_dot = ybd_dot_num / (norm_hat_zbd_xcd * norm_hat_zbd_xcd);
                                        
            // xbd_dot
            Eigen::Vector3d xbd_dot = hatMap(ybd_dot) * zbd + hatMap(ybd) * zbd_dot;
                                        
            // Rd_dot
            Eigen::Matrix3d Rd_dot;
            Rd_dot.col(0) = xbd_dot;
            Rd_dot.col(1) = ybd_dot;
            Rd_dot.col(2) = zbd_dot;
                                        
            // wd and error ew
            Eigen::Matrix3d wd_hat = Rd.transpose() * Rd_dot;
            Eigen::Vector3d wd;
            wd << wd_hat(2,1), wd_hat(0,2), wd_hat(1,0);
            Eigen::Vector3d ew = w - R.transpose() * Rd * wd;
                                        
            // Now compute second derivatives for wd_dot
            // current rotation derivative for zb_dot
            Eigen::Matrix3d R_dot = R * hatMap(w);
            Eigen::Vector3d zb_dot = R_dot.col(2);
                                        
            // u1_dot equivalent
            double u1_dot = Fd_dot.dot(zb) + Fd.dot(zb_dot);
            Eigen::Vector3d current_jerkDes = (u1_dot * zb + f * zb_dot) / m_;
                                        
            Eigen::Vector3d ej = current_jerkDes - snapDes;
                                        
            // Fd_2dot
            Eigen::Vector3d Fd_2dot = -Kp_diag.cwiseProduct(ea) - Kd_diag.cwiseProduct(ej) + m_ * snapDes;
                                        
            // zbd_2dot (following the complicated formula)
            Eigen::Vector3d term1 = Fd_2dot * norm_Fd - Fd * ( (Fd.dot(Fd_2dot)) / norm_Fd );
            Eigen::Vector3d zbd_2dot = term1 / (norm_Fd * norm_Fd)
                - (
                    ( ( (Fd_dot.dot(Fd_dot) + Fd.dot(Fd_2dot)) * Fd
                        + (Fd.dot(Fd_dot)) * Fd_dot ) * (norm_Fd * norm_Fd * norm_Fd)
                      - (Fd.dot(Fd_dot)) * Fd * 3.0 * norm_Fd * (Fd.dot(Fd_dot)) )
                    / (std::pow(norm_Fd, 4))
                  );
              
            // ybd_2dot
            Eigen::Vector3d zbd_x_xcd_2dot = hatMap(zbd_2dot) * xcd
                                            + 2.0 * hatMap(zbd_dot) * xcd_dot
                                            + hatMap(zbd) * xcd_2dot;
              
            // First part ybd_2dot_1
            Eigen::Vector3d ybd_2dot_1_num = (hatMap(zbd_2dot) * xcd + hatMap(zbd) * xcd_2dot) * norm_hat_zbd_xcd
                                             - zbd_x_xcd_dot * zbd_xcd_norm_dot;
            Eigen::Vector3d ybd_2dot_1 = ybd_2dot_1_num / (norm_hat_zbd_xcd * norm_hat_zbd_xcd);
              
            // Compute zbd_xcd_norm_2dot
            double first = zbd_x_xcd_dot.squaredNorm();
            double second = (hatMap(zbd) * xcd).dot(zbd_x_xcd_2dot);
            double zbd_xcd_norm_2dot = ( (first + second) * norm_hat_zbd_xcd
                                        - ( (hatMap(zbd) * xcd).dot(zbd_x_xcd_dot) ) * zbd_xcd_norm_dot )
                                        / (norm_hat_zbd_xcd * norm_hat_zbd_xcd);
              
            // ybd_2dot_2
            Eigen::Vector3d ybd_2dot_2_num = (zbd_x_xcd_dot * zbd_xcd_norm_dot + hatMap(zbd) * xcd * zbd_xcd_norm_2dot)
                                             * (norm_hat_zbd_xcd * norm_hat_zbd_xcd)
                                             - 2.0 * hatMap(zbd) * xcd * norm_hat_zbd_xcd * zbd_xcd_norm_dot * zbd_xcd_norm_dot;
            Eigen::Vector3d ybd_2dot_2 = ybd_2dot_2_num / std::pow(norm_hat_zbd_xcd, 4);
              
            Eigen::Vector3d ybd_2dot = ybd_2dot_1 - ybd_2dot_2;
              
            // xbd_2dot
            Eigen::Vector3d xbd_2dot = hatMap(ybd_2dot) * zbd + 2.0 * hatMap(ybd_dot) * zbd_dot + hatMap(ybd) * zbd_2dot;
              
            // Rd_2dot
            Eigen::Matrix3d Rd_2dot;
            Rd_2dot.col(0) = xbd_2dot;
            Rd_2dot.col(1) = ybd_2dot;
            Rd_2dot.col(2) = zbd_2dot;
              
            Eigen::Matrix3d wd_dot_hat = Rd_dot.transpose() * Rd_dot + Rd.transpose() * Rd_2dot;
            Eigen::Vector3d wd_dot;
            wd_dot << wd_dot_hat(2,1), wd_dot_hat(0,2), wd_dot_hat(1,0);
              
            // Compute torque M
            Eigen::Vector3d KR_diag = KR_.diagonal();
            Eigen::Vector3d KW_diag = KW_.diagonal();
            Eigen::Vector3d M = - KR_diag.cwiseProduct(eR)
                                - KW_diag.cwiseProduct(ew)
                                + hatMap(w) * J_ * w
                                - J_ * ( hatMap(w) * R.transpose() * Rd * wd - R.transpose() * Rd * wd_dot );

            // gzmsg << "Desired rotation:\n" << zbd.cross(xcd).transpose() << "\n";
            // gzmsg << "errorR: " << eR.transpose() << "\nerrorW: " << ew.transpose() << "\n\n";

            // // Torque saturation (optional, mirror original)
            // double max_torque = 0.05;
            // for (int i = 0; i < 3; ++i) {
            //     M(i) = std::max(-max_torque, std::min(max_torque, M(i)));
            // }
        
            Eigen::Vector4d wrench;
            wrench << f, M;
            return wrench;
        }        

    
    private:
        /**
         * @brief The hat map operator.
         * @details Maps a vector $v \in \mathbb{R}^3$ to a skew-symmetric matrix $\hat{v} \in \mathfrak{so}(3)$
         * such that for any vector $u \in \mathbb{R}^3$, $\hat{v}u = v \times u$.
         * @param[in] v A 3D vector.
         * @return The corresponding 3x3 skew-symmetric matrix.
         */
        Eigen::Matrix3d hatMap(const Eigen::Vector3d& v) {
            Eigen::Matrix3d hat;
            hat <<      0,  -v(2),   v(1),
                     v(2),      0,  -v(0),
                    -v(1),   v(0),      0;
            return hat;
        }


        /**
         * @brief The vee map operator, inverse of the hat map.
         * @details Maps a skew-symmetric matrix $A \in \mathfrak{so}(3)$ back to its corresponding vector in $\mathbb{R}^3$.
         * @param[in] A 3x3 skew-symmetric matrix.
         * @return The corresponding 3D vector.
         */
        Eigen::Vector3d veeMap(const Eigen::Matrix3d& A) {
            return Eigen::Vector3d(A(2,1), A(0,2), A(1,0));
        }    


    private:
        // Physical parameters
        double m_;  ///< Mass of the quadrotor ($m$)
        const double g_ = 9.81;  ///< Acceleration due to gravity ($g$)
        Eigen::Matrix3d J_; ///< Inertia matrix of the quadrotor ($J$)

        // Standard basis vectors
        Eigen::Vector3d e1_, e2_, e3_; ///< Standard basis vectors ($e_1, e_2, e_3$)

        // Controller Gains
        Eigen::DiagonalMatrix<double, 3> Kp_; ///< Proportional gains for position ($K_p$)
        Eigen::DiagonalMatrix<double, 3> Kd_; ///< Derivative gains for velocity ($K_d$)
        Eigen::DiagonalMatrix<double, 3> KR_; ///< Proportional gains for attitude ($K_R$)
        Eigen::DiagonalMatrix<double, 3> KW_; ///< Proportional gains for angular velocity ($K_W$)
};

#endif  // GEOMETRIC_CONTROLLER_HPP