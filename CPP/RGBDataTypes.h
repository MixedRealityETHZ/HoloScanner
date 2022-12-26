//
// Created by Zikai Liu on 11/23/22.
//

#pragma once

#include "Timestamp.h"
#include <vector>
#include <DirectXMath.h>

struct XY {
    float x;
    float y;
};

static constexpr size_t RGB_WIDTH = 760;
static constexpr size_t RGB_HEIGHT = 428;

using RGBData = std::vector<uint32_t>;  // row-major order (RGB_WIDTH * RGB_HEIGHT)

struct RGBFrame {
    timestamp_t timestamp;
    RGBData rgb;
    XY focal;
    DirectX::XMMATRIX pv2world;
};

class RGBSource {
public:

    /**
     * Get the RGB camera intrinsics.
     * @param intrinsics  [out] the intrinsics
     * @return true if the intrinsics is available, otherwise the output variable is not touched
     */
    virtual bool getRGBIntrinsics(XY &intrinsics) = 0;

    /**
     * Get the next RGB frame.
     * @param frame      [out] output depth in row-major order (AHAT_WIDTH * AHAT_HEIGHT)
     * @return true if there are new frame(s) available, otherwise the output variables are not touched
     */
    virtual bool getNextAHATFrame(RGBFrame &frame) = 0;
};
