//
// Created by hkcrc-tony on 10/11/23.
//

#include "/home/hkcrc-tony/development/bundle_adjustment/include/bundle_adjustment_param.h"

// 构造函数
BundleAdjustmentParam::BundleAdjustmentParam():CvInternalMatrixRight(3, 3, CV_64F),
                                               CvInternalMatrixLeft(3, 3, CV_64F),
                                               CvExternalMatrixRight(4, 4, CV_64F),
                                               CvExternalMatrixLeft(4, 4, CV_64F),
                                               CvDistortionsLeft(1, 5, CV_64F),
                                               CvDistortionsRight(1, 5, CV_64F),
                                               CvRectifyMatrixLeft(3, 3, CV_64F),
                                               CvRectifyMatrixRight(3, 3, CV_64F),
                                               CvProjectionMatrixLeft(3, 4, CV_64F),
                                               CvProjectionMatrixRight(3, 4, CV_64F),
                                               EigenInternalMatrixRight(),
                                               EigenInternalMatrixLeft(),
                                               EigenDistortionsLeft(),
                                               EigenDistortionsRight(),
                                               EigenRectifyMatrixLeft(),
                                               EigenRectifyMatrixRight(),
                                               EigenProjectionMatrixLeft(),
                                               EigenProjectionMatrixRight(),
                                               FeaturePointRight(),
                                               FeaturePointLeft(),
                                               PriorFeaturePoint(),
                                               EigenExternalMatrixRight(),
                                               EigenExternalMatrixLeft(),
                                               PosteriorFeaturePoint(),
                                               PosteriorCameraPose(),
                                               Inliners()

{
}

// 析构函数
BundleAdjustmentParam::~BundleAdjustmentParam()
{
}

// 获取平差参数的左右相机外参
void BundleAdjustmentParam::GetExternalMatrix(Eigen::Isometry3d &eigenExternalMatrixLeft,
                                              Eigen::Isometry3d &eigenExternalMatrixRight) {
    eigenExternalMatrixLeft = EigenExternalMatrixLeft;
    eigenExternalMatrixRight = EigenExternalMatrixRight;
}

// 获取平差的相机内参
void BundleAdjustmentParam::GetInternalMatrix(cv::Mat &cvInternalMatrixLeft,
                                              cv::Mat &cvInternalMatrixRight,
                                              Eigen::Matrix3d &eigenInternalMatrixLeft,
                                              Eigen::Matrix3d &eigenInternalMatrixRight) {
    cvInternalMatrixLeft = CvInternalMatrixLeft;
    cvInternalMatrixRight = CvInternalMatrixRight;
    eigenInternalMatrixLeft = EigenInternalMatrixLeft;
    eigenInternalMatrixRight = EigenInternalMatrixRight;
}

// 获取相机畸变向量
void BundleAdjustmentParam::GetDistortions(cv::Mat &cvDistortionsLeft, cv::Mat &cvDistortionsRight)
{
    cvDistortionsLeft = CvDistortionsLeft;
    cvDistortionsRight = CvDistortionsRight;
}

// 设置特征点坐标
void BundleAdjustmentParam::SetFeaturePoint(std::vector<cv::Point2f> &featurePointLeft,
                                            std::vector<cv::Point2f> &featurePointRight,
                                            std::vector<Eigen::Vector3d> &priorFeaturePoint) {
    FeaturePointLeft = featurePointLeft;
    FeaturePointRight = featurePointRight;
    PriorFeaturePoint = priorFeaturePoint;
}

// 获取特征点坐标
void BundleAdjustmentParam::GetFeaturePoint(std::vector<cv::Point2f> &featurePointLeft,
                                            std::vector<cv::Point2f> &featurePointRight,
                                            std::vector<Eigen::Vector3d> &featurePoint) {
    featurePointLeft = FeaturePointLeft;
    featurePointRight = FeaturePointRight;
    featurePoint = PriorFeaturePoint;
}

// 获得光束平差结果的函数
void BundleAdjustmentParam::GetBundleAdjustmentResult(std::vector<Eigen::Vector3d> &posteriorFeaturePoint,
                                                      Eigen::Isometry3d &posteriorCameraPose,
                                                      int &inliners) {
    posteriorFeaturePoint = PosteriorFeaturePoint;
    posteriorCameraPose = PosteriorCameraPose;
    inliners = Inliners;
}

// 获取立体校正参数
void BundleAdjustmentParam::GetRectiftyMatrix(Eigen::Matrix3d &eigenRectifyMatrixLeft,
                                              Eigen::Matrix3d &eigenRectifyMatrixRight,
                                              Eigen::Matrix<double, 3, 4> &eigenProjectionMatrixLeft,
                                              Eigen::Matrix<double, 3, 4> &eigenProjectionMatrixRight) {
    eigenProjectionMatrixLeft = EigenProjectionMatrixLeft;
    eigenProjectionMatrixRight = EigenProjectionMatrixRight;
    eigenRectifyMatrixLeft = EigenRectifyMatrixLeft;
    eigenRectifyMatrixRight = EigenRectifyMatrixRight;
}
// 保存光束平差结果的函数
void BundleAdjustmentParam::SaveBundleAdjustmentResult(std::vector<Eigen::Vector3d> &posteriorFeaturePoint,
                                                       Eigen::Isometry3d &posteriorCameraPose,
                                                       int &inliners) {
    PosteriorFeaturePoint = posteriorFeaturePoint;
    PosteriorCameraPose = posteriorCameraPose;
    Inliners = inliners;
}

// 图像去畸变函数
void BundleAdjustmentParam::CorrectDistortion(const cv::Point2f &disortedTarget,
                                              const cv::Mat &cvInternalMatrix,
                                              const cv::Mat &cvDistortions,
                                              cv::Point2f *correctedTarget) {

    // 将目标点的坐标保存在Mat中
    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = disortedTarget.x;
    mat.at<float>(0, 1) = disortedTarget.y;

    // 调整mat的通道为2，矩阵的行列形状不变
    mat=mat.reshape(2);
    cv::undistortPoints(mat,
                        mat,
                        cvInternalMatrix,
                        cvDistortions,
                        cv::Mat(),
                        cvInternalMatrix);
    mat=mat.reshape(1);

    // 存储校正后的目标点
    correctedTarget->x = mat.at<float>(0, 0);
    correctedTarget->y = mat.at<float>(0, 1);
}

// 噪声引入函数
void BundleAdjustmentParam::AddError(const double &errorAngle,
                                     Eigen::Matrix3d &eigenExternalMatrixLeft,
                                     Eigen::Matrix3d &eigenExternalMatrixRight) {
    // 随机数生成器
    std::default_random_engine generator( time(NULL));

    // 随机数的概率分布
    std::uniform_real_distribution<double> unif(-errorAngle, errorAngle);

    // 产生随机数
    double randomError = unif(generator);

    std::cout<<"errorAngle: "<<randomError<<std::endl;

    // ZYX顺序
    Eigen::Vector3d eulerAngleLeft = eigenExternalMatrixLeft.eulerAngles(2,1,0);
    Eigen::Vector3d eulerAngleRight = eigenExternalMatrixRight.eulerAngles(2,1,0);

    // 添加随机误差
    Eigen::Vector3d errorAngleLeft;
    Eigen::Vector3d errorAngleRight;
    errorAngleLeft.x() = eulerAngleLeft.x() + randomError * 3.14159265358 / 180;
    errorAngleLeft.y() = eulerAngleLeft.y() + randomError * 3.14159265358 / 180;
    errorAngleLeft.z() = eulerAngleLeft.z() + randomError * 3.14159265358 / 180;
    errorAngleRight.x() = eulerAngleRight.x() + randomError * 3.14159265358 / 180;
    errorAngleRight.y() = eulerAngleRight.y() + randomError * 3.14159265358 / 180;
    errorAngleRight.z() = eulerAngleRight.z() + randomError * 3.14159265358 / 180;

    // 将欧拉角转化为旋转矩阵
    eigenExternalMatrixLeft = Eigen::AngleAxisd(errorAngleLeft[0], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(errorAngleLeft[1], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(errorAngleLeft[2], Eigen::Vector3d::UnitX());
    eigenExternalMatrixRight = Eigen::AngleAxisd(errorAngleRight[0], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(errorAngleRight[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(errorAngleRight[2], Eigen::Vector3d::UnitX());

}

// 参数文件加载函数
bool BundleAdjustmentParam::LoadFromYamlFile(const std::string &yamlFileName, BundleAdjustmentParam *bundleAdjustmentParam)
{
    // 判断YAML配置文件是否存在
    if (access(yamlFileName.c_str(), F_OK) == -1)
    {
        return false;
    }

    // 判断YAML配置文件是否可读
    if (access(yamlFileName.c_str(), R_OK) == -1)
    {
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        return false;
    }

    // 读取CvInternalMatrixLeft参数
    if ((!fileStorage["CvInternalMatrixLeft"].isNone()) && (fileStorage["CvInternalMatrixLeft"].isMap()))
    {
        fileStorage["CvInternalMatrixLeft"] >> bundleAdjustmentParam -> CvInternalMatrixLeft;
        cv::cv2eigen(bundleAdjustmentParam -> CvInternalMatrixLeft, bundleAdjustmentParam -> EigenInternalMatrixLeft);
    }

    // 读取CvInternalMatrixRight参数
    if ((!fileStorage["CvInternalMatrixRight"].isNone()) && (fileStorage["CvInternalMatrixRight"].isMap()))
    {
        fileStorage["CvInternalMatrixRight"] >> bundleAdjustmentParam -> CvInternalMatrixRight;
        cv::cv2eigen(bundleAdjustmentParam -> CvInternalMatrixRight, bundleAdjustmentParam -> EigenInternalMatrixRight);
    }

    // 读取CvExternalMatrixLeft参数
    if ((!fileStorage["CvExternalMatrixLeft"].isNone()) && (fileStorage["CvExternalMatrixLeft"].isMap()))
    {
        fileStorage["CvExternalMatrixLeft"] >> bundleAdjustmentParam -> CvExternalMatrixLeft;
        cv::cv2eigen(bundleAdjustmentParam -> CvExternalMatrixLeft, bundleAdjustmentParam -> EigenExternalMatrixLeft.matrix());
    }

    // 读取CvExternalMatrixLeft参数
    if ((!fileStorage["CvExternalMatrixRight"].isNone()) && (fileStorage["CvExternalMatrixRight"].isMap()))
    {
        fileStorage["CvExternalMatrixRight"] >> bundleAdjustmentParam -> CvExternalMatrixRight;
        cv::cv2eigen(bundleAdjustmentParam -> CvExternalMatrixRight, bundleAdjustmentParam -> EigenExternalMatrixRight.matrix());
    }

    // 读取CvDistortionsLeft参数
    if ((!fileStorage["CvDistortionsLeft"].isNone()) && (fileStorage["CvDistortionsLeft"].isMap()))
    {
        fileStorage["CvDistortionsLeft"] >> bundleAdjustmentParam -> CvDistortionsLeft;
        cv::cv2eigen(bundleAdjustmentParam -> CvDistortionsLeft, bundleAdjustmentParam -> EigenDistortionsLeft);
    }

    // 读取CvDistortionsRight参数
    if ((!fileStorage["CvDistortionsRight"].isNone()) && (fileStorage["CvDistortionsRight"].isMap()))
    {
        fileStorage["CvDistortionsRight"] >> bundleAdjustmentParam -> CvDistortionsRight;
        cv::cv2eigen(bundleAdjustmentParam -> CvDistortionsRight, bundleAdjustmentParam -> EigenDistortionsRight);
    }

    // 读取CvRectifyMatrixLeft参数
    if ((!fileStorage["CvRectifyMatrixLeft"].isNone()) && (fileStorage["CvRectifyMatrixLeft"].isMap()))
    {
        fileStorage["CvRectifyMatrixLeft"] >> bundleAdjustmentParam -> CvRectifyMatrixLeft;
        cv::cv2eigen(bundleAdjustmentParam -> CvRectifyMatrixLeft, bundleAdjustmentParam -> EigenRectifyMatrixLeft);
    }

    // 读取CvRectifyMatrixRight参数
    if ((!fileStorage["CvRectifyMatrixRight"].isNone()) && (fileStorage["CvRectifyMatrixRight"].isMap()))
    {
        fileStorage["CvRectifyMatrixRight"] >> bundleAdjustmentParam -> CvRectifyMatrixRight;
        cv::cv2eigen(bundleAdjustmentParam -> CvRectifyMatrixRight, bundleAdjustmentParam -> EigenRectifyMatrixRight);
    }

    // 读取CvProjectionMatrixLeft参数
    if ((!fileStorage["CvProjectionMatrixLeft"].isNone()) && (fileStorage["CvProjectionMatrixLeft"].isMap()))
    {
        fileStorage["CvProjectionMatrixLeft"] >> bundleAdjustmentParam -> CvProjectionMatrixLeft;
        cv::cv2eigen(bundleAdjustmentParam -> CvProjectionMatrixLeft, bundleAdjustmentParam -> EigenProjectionMatrixLeft);
    }

    // 读取CvProjectionMatrixRight参数
    if ((!fileStorage["CvProjectionMatrixRight"].isNone()) && (fileStorage["CvProjectionMatrixRight"].isMap()))
    {
        fileStorage["CvProjectionMatrixRight"] >> bundleAdjustmentParam -> CvProjectionMatrixRight;
        cv::cv2eigen(bundleAdjustmentParam -> CvProjectionMatrixRight, bundleAdjustmentParam -> EigenProjectionMatrixRight);
    }

    // 关闭文件存储器
    fileStorage.release();

    return true;
}