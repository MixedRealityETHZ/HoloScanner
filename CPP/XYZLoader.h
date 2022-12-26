//
// Created by Zikai Liu on 12/14/22.
// Open3D header is incompatible with IGL headers so this separate file is needed
//

#ifndef HOLOSCANNER_XYZLOADER_H
#define HOLOSCANNER_XYZLOADER_H

#include <Eigen/Eigen>

Eigen::MatrixXd loadPointCloudFromXYZ(const std::string &fname);

#endif //HOLOSCANNER_XYZLOADER_H
