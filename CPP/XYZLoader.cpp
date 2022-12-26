//
// Created by Zikai Liu on 12/14/22.
//

#include "open3d/Open3D.h"
#include "XYZLoader.h"

Eigen::MatrixXd loadPointCloudFromXYZ(const std::string &fname) {
    open3d::geometry::PointCloud pointCloud;
    open3d::io::ReadPointCloudFromXYZ(fname, pointCloud, {});

    Eigen::MatrixXd mat;
    mat.resize((long) pointCloud.points_.size(), 3);
    for (int i = 0; i < pointCloud.points_.size(); i++) {
        mat.row(i) = pointCloud.points_[i].transpose();
    }

    return mat;
}