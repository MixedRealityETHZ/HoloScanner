//
// Created by Zikai Liu on 11/23/22.
//

#pragma once

#include <DirectXMath.h>

static inline DirectX::XMVECTOR XMTransformToTranslate(const DirectX::XMMATRIX &m) {
    return m.r[3];  // see XMMatrixDecompose
}

static inline float XMVectorSquaredNorm(const DirectX::XMVECTOR &v) {
    DirectX::XMFLOAT3 f;
    DirectX::XMStoreFloat3(&f, v);
    return f.x * f.x + f.y * f.y + f.z * f.z;
}
