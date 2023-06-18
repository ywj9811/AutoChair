#ifndef __BUNDLE_ADJUSTMENT__
#define __BUNDLE_ADJUSTMENT__

#include<iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"

// define ceres-solver header file
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// cost Error Function
struct ReprojectionError {
    // constructor
    ReprojectionError(const cv::Point2d& _x, double _f, const cv::Point2d& _c) : x(_x), f(_f), c(_c) { }

    // error operation
    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const {
        // X' = R * X + t
        T X[3];
        ceres::AngleAxisRotatePoint(camera, point, X);
        // camera = [x, y, z , r1, r2, r3]
        // sum
        X[0] += camera[3];
        X[1] += camera[4];
        X[2] += camera[5];

        // x' = K * X'
        // homogenous coordinate -> pixel coordinate
        T x_p = f * X[0] / X[2] + c.x;
        T y_p = f * X[1] / X[2] + c.y;

        // residual = x - x'
        residuals[0] = T(x.x) - x_p;
        residuals[1] = T(x.y) - y_p;
        return true;
    }

    // cost function 
    static ceres::CostFunction* create(const cv::Point2d& _x, double _f, const cv::Point2d& _c){
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(new ReprojectionError(_x, _f, _c)));
    }

    private:
        const cv::Point2d x;
        const double f;
        const cv::Point2d c;
};

#endif // End of '__BUNDLE_ADJUSTMENT__'

