//
// Created by hkcrc-tony on 23年10月13日.
//

#include "/home/hkcrc-tony/development/bundle_adjustment/include/bundle_adjustment_param.h"

///< 参数加载测试样例
int main()
{
    // 实例化参数类
    BundleAdjustmentParam param;
    std::string yamlFile = "/home/hkcrc-tony/development/bundle_adjustment/config/bundle_adjustment_param.yaml";
    if(BundleAdjustmentParam::LoadFromYamlFile(yamlFile, &param))
    {
        // 打印检验读取的参数
        cv::Mat cvInternalMatrixLeft;
        cv::Mat cvInternalMatrixRight;
        Eigen::Matrix3d eigenInternalMatrixLeft;
        Eigen::Matrix3d eigenInternalMatrixRight;
        Eigen::Isometry3d eigenExternalMatrixLeft;
        Eigen::Isometry3d eigenExternalMatrixRight;

        // 得到左右相机的内外参
        param.GetInternalMatrix(cvInternalMatrixLeft, cvInternalMatrixRight,eigenInternalMatrixLeft, eigenInternalMatrixRight);
        param.GetExternalMatrix(eigenExternalMatrixLeft, eigenExternalMatrixRight);

        // 打印读取的数据
        std::cout<<"InternalMatrixLeft: "<<std::endl<<eigenInternalMatrixLeft<<std::endl;
        std::cout<<"InternalMatrixRight: "<<std::endl<<eigenInternalMatrixRight<<std::endl;
        std::cout<<"ExternalMatrixLeft: "<<std::endl<<eigenExternalMatrixLeft.matrix()<<std::endl;
        std::cout<<"ExternalMatrixRight: "<<std::endl<<eigenExternalMatrixRight.matrix()<<std::endl;
        std::cout<<"load  param successfully"<<std::endl;
    }
}