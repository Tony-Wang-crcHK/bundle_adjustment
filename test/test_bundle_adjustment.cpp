//
// Created by hkcrc-tony on 10/11/23.
//

#include "/home/hkcrc-tony/development/bundle_adjustment/include/bundle_adjustment.h"

///< 光束平差测试样例
int main()
{
    // 实例化参数类, 并读取相机参数
    BundleAdjustmentParam param;
    std::string yamlFile = "/home/hkcrc-tony/development/bundle_adjustment/config/bundle_adjustment_param.yaml";
    if(BundleAdjustmentParam::LoadFromYamlFile(yamlFile, &param))
    {
        std::cout<<"参数加载完成"<<std::endl;
    }

    // 设置特征点坐标
    std::vector<cv::Point2f> featurePointLeft;
    std::vector<cv::Point2f> featurePointRight;
    std::vector<Eigen::Vector3d> featurePoint;

    // 设置左相机特征点像素坐标
    featurePointLeft.emplace_back(cv::Point2f (3131.004, 2646.009));
    featurePointLeft.emplace_back(cv::Point2f (3805.028, 2612.993));
    featurePointLeft.emplace_back(cv::Point2f (3071.951, 2712.993));
    featurePointLeft.emplace_back(cv::Point2f (3808.972, 2676.030));
    featurePointLeft.emplace_back(cv::Point2f (3081.098, 2906.981));
    featurePointLeft.emplace_back(cv::Point2f (3821.973, 2866.006));

    // 设置右相机特征点像素坐标
    featurePointRight.emplace_back(cv::Point2f (1617.917, 2646.000));
    featurePointRight.emplace_back(cv::Point2f (2310.76, 2612.99));
    featurePointRight.emplace_back(cv::Point2f (1553.937, 2713.000));
    featurePointRight.emplace_back(cv::Point2f (2312.51, 2676.03));
    featurePointRight.emplace_back(cv::Point2f (1554.086, 2907.000));
    featurePointRight.emplace_back(cv::Point2f (2318.087, 2866.000));

    // 设置特征点三维坐标
    featurePoint.emplace_back(343.89541704,  757.37516512, 2384.9165071);
    featurePoint.emplace_back(1002.19574058,  734.88103166, 2414.95371827);
    featurePoint.emplace_back(286.38010767,  818.89079529, 2377.17449742);
    featurePoint.emplace_back(1004.54796461,  794.87551627, 2411.41597477);
    featurePoint.emplace_back( 293.3767906,   998.24505248, 2363.16717774);
    featurePoint.emplace_back(1012.12089431,  974.09456501, 2399.50651613);

    // 得到先验的距离
    Eigen::Vector3d priorDistance1 = featurePoint.at(0) - featurePoint.at(1);
    Eigen::Vector3d priorDistance2 = featurePoint.at(2) - featurePoint.at(3);
    Eigen::Vector3d priorDistance3 = featurePoint.at(4) - featurePoint.at(5);

    double distance1 = std::sqrt(priorDistance1.squaredNorm());
    double distance2 = std::sqrt(priorDistance2.squaredNorm());
    double distance3 = std::sqrt(priorDistance3.squaredNorm());

    std::cout<<"priorDistance: "<<std::setprecision(10)<<std::endl<<distance1<<std::endl<<distance2<<std::endl<<distance3<<std::endl;

    // 为参数类设置特征点坐标
    param.SetFeaturePoint(featurePointLeft, featurePointRight, featurePoint);

    // 实例化光束平差类， 并设置参数
    BundleAdjustment stateEstimator;
    stateEstimator.SetPriorCameraParameter(param);

    // 进行状态估计
    stateEstimator.StateEstimation();

    // 得到状态估计结果, 并打印
    std::vector<Eigen::Vector3d> posteriorFeaturePoint;
    Eigen::Isometry3d posteriorCameraPose;
    int inliners;

    stateEstimator.GetPosteriorCameraParameter(param);
    param.GetBundleAdjustmentResult(posteriorFeaturePoint, posteriorCameraPose, inliners);

    Eigen::Vector3d posteriorDistance1 = posteriorFeaturePoint.at(0) - posteriorFeaturePoint.at(1);
    Eigen::Vector3d posteriorDistance2 = posteriorFeaturePoint.at(2) - posteriorFeaturePoint.at(3);
    Eigen::Vector3d posteriorDistance3 = posteriorFeaturePoint.at(4) - posteriorFeaturePoint.at(5);
    double realDistance1 = std::sqrt(posteriorDistance1.squaredNorm());
    double realDistance2 = std::sqrt(posteriorDistance2.squaredNorm());
    double realDistance3 = std::sqrt(posteriorDistance3.squaredNorm());

    std::cout<<"posteriorDistance: "<<std::setprecision(10)<<std::endl<<realDistance1<<std::endl<<realDistance2<<std::endl<<realDistance3<<std::endl;

    for(int i = 0; i < posteriorFeaturePoint.size(); i ++)
    {
        std::cout << "posteriorFeaturePoint: " <<std::endl <<std::setprecision(10)<<posteriorFeaturePoint.at(i) << std::endl;
    }
}
