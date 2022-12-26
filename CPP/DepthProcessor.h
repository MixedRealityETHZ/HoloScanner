//
// Created by Zikai Liu on 11/14/22.
//

#ifndef HOLOSCANNER_DEPTHPROCESSOR_H
#define HOLOSCANNER_DEPTHPROCESSOR_H

#include <queue>
#include <mutex>

#include "RawDataTypes.h"

class DepthProcessor {
public:

    DepthProcessor(const DirectX::XMMATRIX &extrinsics, const float *lut);

    bool update(const RawDataFrame &rawDataFrame);

    bool getNextPCDRaw(timestamp_t &timestamp, PCDRaw &pcdRaw);

    bool getNextHandDebugFrame(timestamp_t &timestamp, HandDebugFrame &hdFrame);

public:

    static constexpr int MAX_PENDING_FRAMES = 3;

    static constexpr size_t ROI_ROW_LOWER = 256 - 168;
    static constexpr size_t ROI_ROW_UPPER = 256 + 80;
    static constexpr size_t ROI_COL_LOWER = 256 - 128;
    static constexpr size_t ROI_COL_UPPER = 256 + 128;
    static constexpr uint16_t DEPTH_NEAR_CLIP = 200;  // Unit: mm
    static constexpr uint16_t DEPTH_FAR_CLIP = 800;
    static constexpr uint16_t DEPTH_FILTER_OFFSET = 200;

    static constexpr size_t CLIPPED_DEPTH_FRAME_WIDTH = ROI_COL_UPPER - ROI_COL_LOWER;
    static constexpr size_t CLIPPED_DEPTH_FRAME_HEIGHT = ROI_ROW_UPPER - ROI_ROW_LOWER;

    std::vector<AHATDepth> clippedDepthFrames;
    std::vector<float> clippedDepthMovingSum;
    size_t stdLogIndex = 0;
    static constexpr size_t STD_LOG_SIZE = 10;

    static constexpr float MAX_STD_VAL = 15000.0f;  // squared value on purpose


    std::queue<std::pair<timestamp_t, PCDRaw>> pcdRawFrames;
    std::queue<HandDebugFrame> handMeshDebugQueue;
    std::mutex pcdMutex;

    DirectX::XMMATRIX extrinsics;
    DirectX::XMMATRIX cam2rig;

    std::vector<DirectX::XMVECTOR> lut;

    Mesh handMesh[HandIndexCount];
    Indices handMeshIndices[HandIndexCount];
    std::vector<float> handFilterDistanceSq[HandIndexCount];  // squared distances

    // @formatter:off
    static constexpr int HAND_BONES[][2] = {
        /* Thumb  */ {1, 2 }, /**/ {2, 3  }, /**/ {3, 4  }, /**/ {4, 5  },
        /* Index  */ {1, 6 }, /**/ {6, 7  }, /**/ {7, 8  }, /**/ {8, 9  }, /**/ {9, 10 },
        /* Middle */ {1, 11}, /**/ {11, 12}, /**/ {12, 13}, /**/ {13, 14}, /**/ {14, 15},
        /* Ring   */ {1, 16}, /**/ {16, 17}, /**/ {17, 18}, /**/ {18, 19}, /**/ {19, 20},
        /* Pinky  */ {1, 21}, /**/ {21, 22}, /**/ {22, 23}, /**/ {23, 24}, /**/ {24, 25}
    };
    // @formatter:on

    static constexpr float WRIST_RADIUS = 0.1f;

    // Normals
    static constexpr float FINGER_RADIUS = 0.025f;
    static constexpr float THUMB_RADIUS = 0.04f;
    static constexpr float INDEX_RADIUS = 0.03f;

    static constexpr float FINGER_SIZES[HandJointIndexCount] = {
            0.05,   /* PALM */
            0.05,   /* WRIST */
            0.05,   /* ThumbMetacarpal */
            0.04,   /* ThumbProximal */
            0.03,   /* ThumbDistal */
            0.025,  /* ThumbTip */
            0.04,   /* IndexMetacarpal */
            0.03,   /* IndexProximal */
            0.02,   /* IndexIntermediate */
            0.02,   /* IndexDistal */
            0.015,  /* IndexTip */
            0.04,   /* MiddleMetacarpal */
            0.02,   /* MiddleProximal */
            0.02,   /* MiddleIntermediate */
            0.02,   /* MiddleDistal */
            0.015,  /* MiddleTip */
            0.04,   /* RingMetacarpal */
            0.02,   /* RingProximal */
            0.02,   /* RingIntermediate */
            0.02,   /* RingDistal */
            0.015,  /* RingTip */
            0.04,   /* PinkyMetacarpal */
            0.02,   /* PinkyProximal */
            0.02,   /* PinkyIntermediate */
            0.02,   /* PinkyDistal */
            0.015,  /* PinkyTip */
    };
    static_assert(sizeof(DepthProcessor::FINGER_SIZES) == HandJointIndexCount * sizeof(float), "FINGER_SIZES");

    static void wristNormals(
            const DirectX::XMVECTOR &wrist,
            const DirectX::XMVECTOR &palm,
            const DirectX::XMVECTOR &indexMetacarpal,
            const DirectX::XMVECTOR &pinkyMetacarpal,
            float dist,
            bool isRightHand,
            DirectX::XMVECTOR &nDir,
            DirectX::XMVECTOR &nTDir,
            std::vector<DirectX::XMVECTOR> &vertices,
            std::vector<std::vector<int>> &indices,
            std::vector<float> &filterDistanceSq
    );

    static void fingerNormals(
            const DirectX::XMVECTOR &tip,
            const DirectX::XMVECTOR &distal,
            const DirectX::XMVECTOR &intermediate,
            const DirectX::XMVECTOR &proximal,
            const DirectX::XMVECTOR &nDir,
            const DirectX::XMVECTOR &nTDir,
            float dist,
            bool flip,
            bool indexOrPinky,
            std::vector<DirectX::XMVECTOR> &vertices,
            std::vector<std::vector<int>> &indices,
            std::vector<float> &filterDistanceSq
    );

    static void thumbNormals(
            const DirectX::XMVECTOR &tip,
            const DirectX::XMVECTOR &distal,
            const DirectX::XMVECTOR &proximal,
            const DirectX::XMVECTOR &nTDir,
            float dist,
            bool isRightHand,
            std::vector<DirectX::XMVECTOR> &vertices,
            std::vector<std::vector<int>> &indices,
            std::vector<float> &filterDistanceSq
    );

    static void updateHandMesh(
            const Hand &hand,
            std::vector<DirectX::XMVECTOR> &vertices,
            std::vector<std::vector<int>> &indices,
            std::vector<float> &filterDistanceSq
    );

    bool inHandMesh(const DirectX::XMVECTOR &point);
};


#endif //HOLOSCANNER_DEPTHPROCESSOR_H
