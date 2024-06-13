//
// Created by hkcrc-tony on 10/11/23.
//

#ifndef BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_H

#include <chrono>
#include <iostream>

#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "bundle_adjustment_param.h"

///< 光束平差类
class BundleAdjustment {
public:

    // EIGEN数据结构对齐的宏
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 构造函数
     */
    BundleAdjustment();

    /**
     * @brief 析构函数
     */
    ~BundleAdjustment();

    /**
     * @brief 状态估计函数
     * @note 此处为通过g2o实现的BA
     */
    void StateEstimation();

    /**
     * @brief 设置先验参数函数
     */
    void SetPriorCameraParameter(BundleAdjustmentParam &priorCameraParameter);

    /**
     * @brief 得到后验参数函数
     */
    void GetPosteriorCameraParameter(BundleAdjustmentParam &posteriorCameraParameter);

private:

    BundleAdjustmentParam PriorCameraParameter;                                           ///< 先验相机参数
    BundleAdjustmentParam PosteriorCameraParameter;                                       ///< 后验相机参数
};

#endif //BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_H
