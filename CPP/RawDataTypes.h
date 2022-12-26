//
// Created by Zikai Liu on 11/14/22.
//

#pragma once

#include "Timestamp.h"
#include <vector>
#include <DirectXMath.h>

/// AHAT

static constexpr size_t AHAT_WIDTH = 512;
static constexpr size_t AHAT_HEIGHT = 512;

using AHATLUT = std::vector<float>;       // row-major order (AHAT_WIDTH * AHAT_HEIGHT)
using AHATDepth = std::vector<uint16_t>;  // row-major order (AHAT_WIDTH * AHAT_HEIGHT)

/// Interaction

struct HandJoint {
    bool tracked;
    DirectX::XMMATRIX transformationInWorld;  // world coordinate
    DirectX::XMVECTOR translationInWorld;     // world coordinate
};

enum HandJointIndex {
    Palm,
    Wrist,
    ThumbMetacarpal,
    ThumbProximal,
    ThumbDistal,
    ThumbTip,
    IndexMetacarpal,
    IndexProximal,
    IndexIntermediate,
    IndexDistal,
    IndexTip,
    MiddleMetacarpal,
    MiddleProximal,
    MiddleIntermediate,
    MiddleDistal,
    MiddleTip,
    RingMetacarpal,
    RingProximal,
    RingIntermediate,
    RingDistal,
    RingTip,
    PinkyMetacarpal,
    PinkyProximal,
    PinkyIntermediate,
    PinkyDistal,
    PinkyTip,
    HandJointIndexCount,
};

struct Hand {
    bool tracked;
    bool strictlyTracked;  // hand tracked and all joints tracked
    HandJoint joints[HandJointIndexCount];
};

enum HandIndex {
    Left,
    Right,
    HandIndexCount,
};

struct RawDataFrame {
    timestamp_t timestamp;

    AHATDepth depth;

    DirectX::XMMATRIX rig2world;

    DirectX::XMMATRIX headTransformation;  // world coordinate

    Hand hands[HandIndexCount];

    bool eyeTracked;
    DirectX::XMVECTOR eyeOrigin;     // world coordinate
    DirectX::XMVECTOR eyeDirection;  // world coordinate
};

using Mesh = std::vector<DirectX::XMVECTOR>;
using Indices = std::vector<std::vector<int>>;

struct HandDebugFrame {
    timestamp_t timestamp;
    Mesh lhMesh;
    Indices lhIndices;
    Mesh rhMesh;
    Indices rhIndices;
};

class RawDataSource {
public:

    /**
     * Get the AHAT extrinsics matrix.
     * @param extrinsics  [out] the extrinsics matrix
     * @return true if the extrinsics is available, otherwise the output variable is not touched
     */
    virtual bool getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) = 0;

    /**
     * Get the AHAT depth LUT (x/y/z coefficients to be timed with depth at each pixel to get the 3D point)
     * @param lut  [out] buffer of LUT in flatten row-major order (3 * AHAT_WIDTH * AHAT_HEIGHT)
     * @return true if the LUT is available, otherwise the output variable is not touched
     */
    virtual bool getAHATDepthLUT(AHATLUT &lut) = 0;

    /**
     * Get the next frame of depth, rig2world and interaction.
     * @param frame      [out] output depth in
     * @return true if there are new frame available, otherwise the output variables are not touched
     */
    virtual bool getNextRawDataFrame(RawDataFrame& frame) = 0;

protected:

    static int countTrackedJoints(const Hand &hand) {
        int result = 0;
        for (const auto &joint: hand.joints) {
            if (joint.tracked) result++;
        }
        return result;
    }
};

using PCDRaw = std::vector<float>;
