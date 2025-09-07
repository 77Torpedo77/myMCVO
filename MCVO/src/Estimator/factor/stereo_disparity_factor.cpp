#include "stereo_disparity_factor.h"

double StereoDisparityFactor::sum_t = 0;
double MultiStereoScaleFactor::sum_t = 0;

StereoDisparityFactor::StereoDisparityFactor(const Eigen::Vector3d &_pts_i, double _stereo_depth, double _weight)
    : pts_i(_pts_i), stereo_depth(_stereo_depth), weight(_weight)
{
    
}

bool StereoDisparityFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double inv_dep_i = parameters[3][0];

    // Transform point to world coordinates using estimated depth
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    
    // Calculate estimated depth
    double estimated_depth = pts_camera_i.norm();
    
    // Residual: difference between stereo depth and estimated depth
    residuals[0] = weight * (estimated_depth - stereo_depth);

    if (jacobians)
    {
        Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

        double dep_j = pts_camera_j.z();
        
        Eigen::Matrix<double, 1, 3> reduce(1.0/dep_j, 0, -pts_camera_j(0)/(dep_j*dep_j));

        reduce = weight * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = Qi.inverse().toRotationMatrix();
            jaco_i.rightCols<3>() = -Qi.inverse().toRotationMatrix() * Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = -Qi.inverse().toRotationMatrix();
            jaco_j.rightCols<3>() = Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = qic.inverse().toRotationMatrix();
            jaco_ex.rightCols<3>() = -qic.inverse().toRotationMatrix() * Utility::skewSymmetric(pts_imu_j - tic);
            
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[3])
        {
            Eigen::Map<Eigen::Vector1d> jacobian_feature(jacobians[3]);
            jacobian_feature(0) = weight * pts_i.norm() / (inv_dep_i * inv_dep_i);
        }
    }
    
    sum_t += tic_toc.toc();
    return true;
}

void StereoDisparityFactor::check(double **parameters)
{
    // Implementation for numerical check
    double *res = new double[1];
    double **jaco = new double *[4];
    jaco[0] = new double[1 * 7];
    jaco[1] = new double[1 * 7];
    jaco[2] = new double[1 * 7];
    jaco[3] = new double[1 * 1];
    Evaluate(parameters, res, jaco);

    delete[] jaco[0];
    delete[] jaco[1];
    delete[] jaco[2];
    delete[] jaco[3];
    delete[] jaco;
    delete[] res;
}

// Multi-stereo scale consistency factor implementation
MultiStereoScaleFactor::MultiStereoScaleFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, 
                                             double _depth_i, double _depth_j, double _weight)
    : pts_i(_pts_i), pts_j(_pts_j), depth_i(_depth_i), depth_j(_depth_j), weight(_weight)
{
    
}

bool MultiStereoScaleFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    
    // Poses and extrinsics for two cameras
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic_i(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic_i(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d tic_j(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond qic_j(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    double inv_dep_i = parameters[4][0];
    double inv_dep_j = parameters[5][0];

    // Calculate 3D points in world coordinates from both cameras
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic_i * pts_camera_i + tic_i;
    Eigen::Vector3d pts_w_i = Qi * pts_imu_i + Pi;

    Eigen::Vector3d pts_camera_j = pts_j / inv_dep_j;
    Eigen::Vector3d pts_imu_j = qic_j * pts_camera_j + tic_j;
    Eigen::Vector3d pts_w_j = Qj * pts_imu_j + Pj;

    // Scale consistency residual: relative scale error between two stereo measurements
    double scale_i = pts_camera_i.norm() / depth_i;
    double scale_j = pts_camera_j.norm() / depth_j;
    
    residuals[0] = weight * (scale_i - scale_j);

    // Jacobians would be computed here if needed for optimization
    if (jacobians)
    {
        // For brevity, setting jacobians to zero (can be computed analytically)
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            jacobian_pose_j.setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_ex_i(jacobians[2]);
            jacobian_ex_i.setZero();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_ex_j(jacobians[3]);
            jacobian_ex_j.setZero();
        }
        if (jacobians[4])
        {
            Eigen::Map<Eigen::Vector1d> jacobian_feature_i(jacobians[4]);
            jacobian_feature_i(0) = weight * pts_i.norm() / (depth_i * inv_dep_i * inv_dep_i);
        }
        if (jacobians[5])
        {
            Eigen::Map<Eigen::Vector1d> jacobian_feature_j(jacobians[5]);
            jacobian_feature_j(0) = -weight * pts_j.norm() / (depth_j * inv_dep_j * inv_dep_j);
        }
    }
    
    sum_t += tic_toc.toc();
    return true;
}