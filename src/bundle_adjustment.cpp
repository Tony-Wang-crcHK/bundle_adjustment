//
// Created by hkcrc-tony on 10/11/23.
//

#include "/home/hkcrc-tony/development/bundle_adjustment/include/bundle_adjustment.h"

// 构造函数
BundleAdjustment::BundleAdjustment():PriorCameraParameter(),
                                     PosteriorCameraParameter()
{
}

// 析构函数
BundleAdjustment::~BundleAdjustment()
{
}

// 状态估计实现
void BundleAdjustment::StateEstimation()
{
    // 下为g2o求解部分的标准步骤
    // 设置矩阵块（维度为动态）
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> block;

    // g2o的第一个步骤，选择一个线性方程求解器
    std::unique_ptr<block::LinearSolverType> linearSolver(new g2o::LinearSolverCholmod<block::PoseMatrixType>());

    // g2o的第二个步骤，选择一个稀疏矩阵块求解器
    std::unique_ptr<block> solverPtr(new block(std::move(linearSolver)));

    // g2o的第二个步骤，选择一个梯度下降方法，此处选择的是LM，实测LM的效果最好
    // 还可换成 OptimizationAlgorithmGaussNewton与 OptimizationAlgorithmDogleg， 即为GN和DogLeg
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solverPtr));

    // 设置核心维护的优化器， 并采用上述设置的求解器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // 初始化运算所需参数
    // http://docs.ros.org/en/fuerte/api/re_vision/html/classg2o_1_1SE3Quat.html
    std::vector<cv::Point2f> featurePointLeft;
    std::vector<cv::Point2f> featurePointRight;
    std::vector<Eigen::Vector3d> featurePoint;

    // 初始化运算所需参数
    cv::Mat cvDistortionsLeft;
    cv::Mat cvDistortionsRight;
    cv::Mat cvInternalMatrixLeft;
    cv::Mat cvInternalMatrixRight;

    // 初始化运算所需参数
    Eigen::Isometry3d eigenExternalMatrixLeft;
    Eigen::Isometry3d eigenExternalMatrixRight;
    Eigen::Matrix3d eigenInternalMatrixLeft;
    Eigen::Matrix3d eigenInternalMatrixRight;
    Eigen::Matrix3d eigenRectifyMatrixLeft;
    Eigen::Matrix3d eigenRectifyMatrixRight;
    Eigen::Matrix<double, 3, 4> eigenProjectionMatrixLeft;
    Eigen::Matrix<double, 3, 4> eigenProjectionMatrixRight;

    // 从参数类中获取参数
    PriorCameraParameter.GetDistortions(cvDistortionsLeft, cvDistortionsRight);
    PriorCameraParameter.GetExternalMatrix(eigenExternalMatrixLeft, eigenExternalMatrixRight);
    PriorCameraParameter.GetFeaturePoint(featurePointLeft, featurePointRight, featurePoint);
    PriorCameraParameter.GetInternalMatrix(cvInternalMatrixLeft, cvInternalMatrixRight, eigenInternalMatrixLeft, eigenInternalMatrixRight);
    PriorCameraParameter.GetRectiftyMatrix(eigenRectifyMatrixLeft, eigenRectifyMatrixRight, eigenProjectionMatrixLeft, eigenProjectionMatrixRight);

    // 得到两相机在纠正后的位姿变换矩阵
    Eigen::Matrix3d rotationLeft = eigenRectifyMatrixLeft.inverse() * eigenExternalMatrixLeft.rotation();
    Eigen::Matrix3d rotationRight = eigenRectifyMatrixRight.inverse() * eigenExternalMatrixRight.rotation();
    Eigen::Vector3d translationLeft = eigenRectifyMatrixLeft.inverse() * eigenExternalMatrixLeft.translation();
    Eigen::Vector3d translationRight = eigenRectifyMatrixRight.inverse() * eigenExternalMatrixRight.translation();

    // 加入随机误差
    // PriorCameraParameter.AddError(10, rotationRight, rotationRight);

    g2o::SE3Quat poseSE3L(rotationLeft,translationLeft);
    g2o::SE3Quat poseSE3R(rotationRight,translationRight);

    // 得到左相机 X 与 Y 方向的偏移量
    double offsetXL = eigenProjectionMatrixLeft(0, 2);
    double offsetYL = eigenProjectionMatrixLeft(1, 2);

    // 得到左相机的焦距
    double focalLengthL = eigenProjectionMatrixLeft(0, 0);

    // 得到右相机的偏移量
    double offsetXR = eigenProjectionMatrixRight(0, 2);
    double offsetYR = eigenProjectionMatrixRight(1, 2);

    // 得到右相机的焦距
    double focalLengthR = eigenProjectionMatrixRight(0, 0);

    int pointSize = featurePoint.size();

    // 添加左相机位姿节点, 固定此相机位姿，意为不参加优化
    g2o::VertexSE3Expmap *cameraLeft = new g2o::VertexSE3Expmap();
    cameraLeft -> setId(0);
    cameraLeft -> setFixed(true);
    cameraLeft -> setEstimate(poseSE3L);
    optimizer.addVertex(cameraLeft);

    // 添加右相机位姿节点
    g2o::VertexSE3Expmap *cameraRight = new g2o::VertexSE3Expmap();
    cameraRight -> setId(1);
    cameraRight -> setEstimate(poseSE3R);
    optimizer.addVertex(cameraRight);

    // 添加先验特征点坐标作为节点
    for(int i = 0; i < pointSize; i++)
    {
        g2o::VertexPointXYZ *priorPoints = new g2o::VertexPointXYZ;
        priorPoints ->setId(2 + i);
        Eigen::Vector3d priorCoordinate = featurePoint.at(i);

        // 设置节点边缘化, 降低计算压力
        priorPoints -> setMarginalized(true);
        priorPoints -> setEstimate(priorCoordinate);
        optimizer.addVertex(priorPoints);
    }

    // 引入左相机
    g2o::CameraParameters *cameraL = new g2o::CameraParameters(focalLengthL , Eigen::Vector2d(offsetXL, offsetYL), 0);
    cameraL->setId(0);
    optimizer.addParameter(cameraL);

    // 引入右相机
    g2o::CameraParameters *cameraR = new g2o::CameraParameters(focalLengthR , Eigen::Vector2d(offsetXR, offsetYR), 0);
    cameraR->setId(1);
    optimizer.addParameter(cameraR);

    // 添加第一帧的边
    std::vector<g2o::EdgeProjectXYZ2UV *> edgesVector;
    for (int i = 0; i < pointSize; i++)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();

        // 导入边的两个节点，一个节点为三维特征点坐标， 一个为左相机位姿
        edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(i + 2)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0)));

        // 导入测量值为特征点像素坐标
        edge->setMeasurement(Eigen::Vector2d(featurePointLeft.at(i).x, featurePointLeft.at(i).y));

        // 设置边信息矩阵，边信息矩阵的作用是衡量每个误差项的重要性和精度，从而指导优化算法的迭代过程。信息矩阵的对角线元素越大，表示对应误差项的精度越高，
        // 优化时应该更加重视这个误差项。因此，将信息矩阵设置为单位矩阵，表示所有误差项的精度都相等，优化时不会对任何一个误差项进行更多的迭代，而是平均分配迭代次数。
        edge->setInformation(Eigen::Matrix2d::Identity());

        // 设置相机参数，为0号相机
        edge->setParameterId(0, 0);

        // 设置核函数，为cauchy核函数
        edge->setRobustKernel(new g2o::RobustKernelCauchy);
        optimizer.addEdge(edge);
        edgesVector.push_back(edge);
    }

    // 添加第二帧的边
    for (size_t i = 0; i < pointSize; i++)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();

        // 导入边的两个节点，一个节点为三维特征点坐标， 一个为右相机位姿
        edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(i + 2)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1)));

        // 导入测量值为特征点像素坐标
        edge->setMeasurement(Eigen::Vector2d(featurePointRight.at(i).x, featurePointRight.at(i).y));

        // 设置边信息矩阵
        edge->setInformation(Eigen::Matrix2d::Identity());

        // 设置相机参数，为1号相机
        edge->setParameterId(0, 1);

        // 设置核函数，为cauchy核函数
        edge->setRobustKernel(new g2o::RobustKernelCauchy);
        optimizer.addEdge(edge);
        edgesVector.push_back(edge);
    }

    // 进行优化, 并且迭代20次
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // 得到优化后的右相机位姿
    g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(1));
    Eigen::Isometry3d cameraPoseR = v->estimate();

    // 得到优化后的特征点坐标
    std::vector<Eigen::Vector3d> pointPose3D;
    for (size_t i = 0; i < pointSize; i++)
    {
        Eigen::Vector3d pointPose;
        g2o::VertexPointXYZ *featurePoint3D = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(i + 2));
        pointPose = featurePoint3D -> estimate();
        pointPose3D.emplace_back(pointPose);
    }

    // chi2表示优化过程中每个误差项的平方残差， g2o会将所有误差项的平方残差加起来作为一个总的平方残差，即为chi2， 越小优化结果越好
    int inliners = 0;
    int outliners = 0;
    for (auto qualified:edgesVector)
    {
        qualified -> computeError();

        // chi2()如果很大，说明此边的值拉高了整体的平方残差和，因此可以认为此边误差较大
        if (qualified -> chi2() > 1)
        {
            outliners++;
        }
        else
        {
            inliners++;
        }
    }

    // 保存光束法平差的结果
    PosteriorCameraParameter.SaveBundleAdjustmentResult(pointPose3D, cameraPoseR, inliners);
    optimizer.save("/home/hkcrc-tony/development/bundle_adjustment/data/ba.g2o");
}

// 设置先验参数函数
void BundleAdjustment::SetPriorCameraParameter(BundleAdjustmentParam &priorCameraParameter)
{
    PriorCameraParameter = priorCameraParameter;
}

// 读取后验参数函数
void BundleAdjustment::GetPosteriorCameraParameter(BundleAdjustmentParam &posteriorCameraParameter)
{
    posteriorCameraParameter = PosteriorCameraParameter;
}