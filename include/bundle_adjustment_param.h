//
// Created by hkcrc-tony on 10/11/23.
//

#ifndef BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_PARAM_H
#define BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_PARAM_H

#include <stdio.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

///< 光束平差参数类
class BundleAdjustmentParam {
public:

    /**
     * @brief 构造函数
     */
    BundleAdjustmentParam();

    /**
     * @brief 析构函数
     */
    ~BundleAdjustmentParam();

    /**
     * @brief 获取优化前外参的函数
     */
    void GetExternalMatrix(Eigen::Isometry3d &eigenExternalMatrixLeft,
                           Eigen::Isometry3d &eigenExternalMatrixRight);

    /**
     * @brief 获取优化前內参的函数
     */
    void GetInternalMatrix(cv::Mat &cvInternalMatrixLeft,
                           cv::Mat &cvInternalMatrixRight,
                           Eigen::Matrix3d &eigenInternalMatrixLeft,
                           Eigen::Matrix3d &eigenInternalMatrixRight);

    /**
     * @brief 获取优化前內参的函数
     */
    void GetDistortions(cv::Mat &cvDistortionsLeft,
                        cv::Mat &cvDistortionsRight);

    /**
     * @brief 设置优化前特征点坐标的函数
     */
    void SetFeaturePoint(std::vector<cv::Point2f> &featurePointLeft,
                         std::vector<cv::Point2f> &featurePointRight,
                         std::vector<Eigen::Vector3d> &priorFeaturePoint);

    /**
     * @brief 获取优化前特征点坐标的函数
     */
    void GetFeaturePoint(std::vector<cv::Point2f> &featurePointLeft,
                         std::vector<cv::Point2f> &featurePointRight,
                         std::vector<Eigen::Vector3d> &featurePoint);

    /**
     * @brief 获得光束平差结果的函数
     */
    void GetBundleAdjustmentResult(std::vector<Eigen::Vector3d> &posteriorFeaturePoint,
                                   Eigen::Isometry3d &posteriorCameraPose,
                                   int &inliners);

    /**
     * @brief 获得立体校正结果的函数
     */
    void GetRectiftyMatrix(Eigen::Matrix3d &eigenRectifyMatrixLeft,
                           Eigen::Matrix3d &eigenRectifyMatrixRight,
                           Eigen::Matrix<double, 3, 4> &eigenProjectionMatrixLeft,
                           Eigen::Matrix<double, 3, 4> &eigenProjectionMatrixRight);

    /**
     * @brief 保存光束平差结果的函数
     */
    void SaveBundleAdjustmentResult(std::vector<Eigen::Vector3d> &posteriorFeaturePoint,
                                    Eigen::Isometry3d &posteriorCameraPose,
                                    int &inliners);

    /**
     * @brief 目标点去畸变函数
     * @note 去畸变相关: https://zhuanlan.zhihu.com/p/137053640 \n
     */
    void CorrectDistortion(const cv::Point2f &disortedTarget,
                           const cv::Mat &cvInternalMatrix,
                           const cv::Mat &cvDistortions,
                           cv::Point2f *correctedTarget);

    /**
     * @brief 引入测试噪声的函数
     */
    void AddError(const double &errorAngle,
                  Eigen::Matrix3d &eigenExternalMatrixLeft,
                  Eigen::Matrix3d &eigenExternalMatrixRight);

    /**
     * @brief 参数文件加载的函数
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, BundleAdjustmentParam *bundleAdjustmentParam);

private:

    // 系统加载参数
    cv::Mat CvInternalMatrixLeft;                                                                    ///< openCV格式的左相机内参
    cv::Mat CvInternalMatrixRight;                                                                   ///< openCV格式的右相机内参
    cv::Mat CvExternalMatrixLeft;                                                                    ///< openCV格式的左相机外参
    cv::Mat CvExternalMatrixRight;                                                                   ///< openCV格式的右相机外参
    cv::Mat CvDistortionsLeft;                                                                       ///< openCV格式的左相机畸变系数向量
    cv::Mat CvDistortionsRight;                                                                      ///< openCV格式的右相机畸变系数向量
    cv::Mat CvRectifyMatrixLeft;                                                                     ///< openCV格式的左相机校正矩阵
    cv::Mat CvRectifyMatrixRight;                                                                    ///< openCV格式的右相机校正矩阵
    cv::Mat CvProjectionMatrixLeft;                                                                  ///< openCV格式的左相机投影矩阵
    cv::Mat CvProjectionMatrixRight;                                                                 ///< openCV格式的右相机投影矩阵

    // 系统运算参数
    Eigen::Matrix3d EigenInternalMatrixLeft;                                                         ///< Eigen格式的左相机内参矩阵
    Eigen::Matrix3d EigenInternalMatrixRight;                                                        ///< Eigen格式的右相机内参矩阵
    Eigen::Matrix<double, 5, 1> EigenDistortionsLeft;                                                ///< Eigen格式的左相机畸变系数向量
    Eigen::Matrix<double, 5, 1> EigenDistortionsRight;                                               ///< Eigen格式的右相机畸变系数向量
    Eigen::Matrix3d EigenRectifyMatrixLeft;                                                          ///< Eigen格式的左相机校正矩阵
    Eigen::Matrix3d EigenRectifyMatrixRight;                                                         ///< Eigen格式的右相机校正矩阵
    Eigen::Matrix<double, 3, 4> EigenProjectionMatrixLeft;                                           ///< Eigen格式的左相机投影矩阵
    Eigen::Matrix<double, 3, 4> EigenProjectionMatrixRight;                                          ///< Eigen格式的右相机投影矩阵

    // 待优化参数
    std::vector<cv::Point2f> FeaturePointLeft;                                                       ///< 左图像特征点坐标
    std::vector<cv::Point2f> FeaturePointRight;                                                      ///< 右图像特征点坐标
    std::vector<Eigen::Vector3d> PriorFeaturePoint;                                                  ///< 先验特征点坐标
    Eigen::Isometry3d EigenExternalMatrixLeft;                                                       ///< Eigen格式的左相机外参矩阵
    Eigen::Isometry3d EigenExternalMatrixRight;                                                      ///< Eigen格式的右相机外参矩阵

    // 优化结果
    std::vector<Eigen::Vector3d> PosteriorFeaturePoint;                                               ///< 优化后的特征点三维坐标
    Eigen::Isometry3d PosteriorCameraPose;                                                            ///< 优化后的相机位姿
    int Inliners;                                                                                     ///< 优化后误差合规的边数
};
#endif //BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_PARAM_H
