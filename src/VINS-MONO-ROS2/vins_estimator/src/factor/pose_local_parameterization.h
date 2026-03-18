#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include "../utility/utility.h"

class PoseLocalParameterization : public ceres::Manifold
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    virtual bool PlusJacobian(const double *x, double *jacobian) const override;
    virtual bool MinusJacobian(const double *x, double *jacobian) const override;
    virtual int AmbientSize() const override { return 7; };
    virtual int TangentSize() const override { return 6; };
};
