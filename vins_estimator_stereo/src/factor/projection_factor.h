#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionFactor : public ceres::SizedCostFunction<4, 7, 7, 7, 1>
{
  public:
  ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
  ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
		   const Eigen::Vector3d &_pts_i_r, const Eigen::Vector3d &_pts_j_r);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j, pts_i_r, pts_j_r;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix4d sqrt_info;
    static double sum_t;


};
