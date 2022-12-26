//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include "Timestamp.h"
#include <vector>
#include <Eigen/Eigen>
#include <DirectXMath.h>

using PCD = std::vector<Eigen::Vector3d>;  // point cloud data (Open3D use Vector3d not Vector3f)

class PCDSource {
public:

    virtual bool getNextPCD(timestamp_t &timestamp, PCD &pcd) = 0;
};