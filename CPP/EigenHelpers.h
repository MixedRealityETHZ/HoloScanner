//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include <Eigen/Eigen>
#include <DirectXMath.h>

static inline Eigen::Vector3d XMVectorToEigenVector3d(const DirectX::XMVECTOR &v) {
    DirectX::XMFLOAT4 f;
    DirectX::XMStoreFloat4(&f, v);
    // NOTICE: x ang y swapped and all negated
    return Eigen::Vector3d{f.x, f.y, f.z} / f.w;
}

static inline DirectX::XMVECTOR EigenVector3dToXMVector(const Eigen::Vector3d &v) {
    // NOTICE: x ang y swapped and all negated
    DirectX::XMFLOAT4 f{(float) v(0), (float) v(1), (float) v(2), 1.0f};
    return DirectX::XMLoadFloat4(&f);
}

