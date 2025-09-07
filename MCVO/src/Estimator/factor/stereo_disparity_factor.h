#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../../utility/utility.h"
#include "../../utility/tic_toc.h"
#include "../parameters.h"

// Stereo disparity constraint factor
// Constrains the depth estimated from stereo disparity vs triangulated depth
class StereoDisparityFactor: public ceres::SizedCostFunction<1, 7, 7, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoDisparityFactor(const Eigen::Vector3d &_pts_i, double _stereo_depth, double _weight = 1.0);

    virtual bool
    Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);

public:
    Eigen::Vector3d pts_i;      // normalized point in host frame
    double stereo_depth;        // depth from stereo disparity 
    double weight;              // weight for this constraint
    static double sum_t;
};

// Multi-stereo scale consistency factor
// Ensures scale consistency across multiple stereo cameras
class MultiStereoScaleFactor: public ceres::SizedCostFunction<1, 7, 7, 7, 7, 1, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MultiStereoScaleFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, 
                          double _depth_i, double _depth_j, double _weight = 1.0);

    virtual bool
    Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

public:
    Eigen::Vector3d pts_i, pts_j;  // normalized points in two cameras
    double depth_i, depth_j;       // stereo depths from two cameras
    double weight;                 // weight for this constraint
    static double sum_t;
};