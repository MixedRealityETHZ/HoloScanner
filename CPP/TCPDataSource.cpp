//
// Created by Zikai Liu on 11/12/22.
//

#include <iostream>
#include <ctime>
#include <utility>
#include "DirectXHelpers.h"
#include "EigenHelpers.h"
#include "TCPDataSource.h"

TCPDataSource::TCPDataSource(std::function<void()> stopCallBack) :
        socketServer(tcpIOContext, PORT, [](auto s) {
            // Setup a server with automatic acceptance
            std::cout << "TerminalSocketServer: disconnected" << std::endl;
            s->startAccept();
        }),
        tcpIOThread(new std::thread([this] {
            boost::asio::executor_work_guard<boost::asio::io_context::executor_type> workGuard(
                    tcpIOContext.get_executor());
            tcpIOContext.run();  // this operation is blocking, until ioContext is deleted
        })),
        stopCallBack(std::move(stopCallBack)){

    socketServer.startAccept();
    socketServer.setCallbacks(nullptr,
                              nullptr,
                              [this](auto name, auto buf, auto size) { handleRecvBytes(name, buf, size); },
                              nullptr);
}

TCPDataSource::~TCPDataSource() {
    if (tcpIOThread) tcpIOThread->join();
}

bool TCPDataSource::sendReconstructedPCD(const Eigen::RowVector3d &pointColor, const Eigen::MatrixXd &pcd) {
    std::vector<float> data;
    data.reserve(3 + pcd.rows() * 3);
    data.emplace_back((float) pointColor(0));
    data.emplace_back((float) pointColor(1));
    data.emplace_back((float) pointColor(2));
    for (int i = 0; i < pcd.rows(); i++) {
        // EigenVector3dToXMVector, but -z
        data.emplace_back((float) pcd(i, 0));
        data.emplace_back((float) pcd(i, 1));
        data.emplace_back((float) -pcd(i, 2));
    }
    socketServer.sendBytes("P", reinterpret_cast<uint8_t *>(data.data()), data.size() * sizeof(float));
    return true;
}

bool TCPDataSource::getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) {
    std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
    if (!ahatExtrinsicsValid) return false;
    extrinsics = ahatExtrinsics;
    return true;
}

bool TCPDataSource::getAHATDepthLUT(AHATLUT &lut) {
    std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
    if (ahatLUT.empty()) return false;
    lut = ahatLUT;
    return true;
}

bool TCPDataSource::getNextRawDataFrame(RawDataFrame &frame) {
    std::lock_guard<std::mutex> lock(rawDataFrameMutex);
    if (rawDataFrames.empty()) return false;
    frame = std::move(rawDataFrames.front());
    rawDataFrames.pop();
    return true;
}

bool TCPDataSource::getNextPCD(timestamp_t &timestamp, PCD &pcd) {
    std::lock_guard<std::mutex> lock(pcdMutex);
    if (pcdFrames.empty()) return false;
    timestamp = pcdFrames.front().first;
    pcd = std::move(pcdFrames.front().second);
    pcdFrames.pop();
    return true;
}

void TCPDataSource::handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size) {

    if (name == "s") {  // Stop signal
        static constexpr size_t EXPECTED_SIZE = 0;
        if (size != EXPECTED_SIZE) {
            std::cerr << "Invalid Stop signal. Size: " << size << ", expecting " << EXPECTED_SIZE << std::endl;
            return;
        }

        stopCallBack();
    } else if (name == "e") {  // AHAT extrinsics
        static constexpr size_t EXPECTED_SIZE = 16 * sizeof(float);
        if (size != EXPECTED_SIZE) {
            std::cerr << "Invalid AHAT extrinsics size: " << size << ", expecting " << EXPECTED_SIZE << std::endl;
            return;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
            ahatExtrinsics = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) buf);
            ahatExtrinsicsValid = true;
        }

    } else if (name == "l") {  // AHAT LUT
        static constexpr size_t EXPECTED_SIZE = 3 * AHAT_WIDTH * AHAT_HEIGHT * sizeof(float);
        if (size != EXPECTED_SIZE) {
            std::cerr << "Invalid AHAT LUT size: " << size << ", expecting " << EXPECTED_SIZE << std::endl;
            return;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(ahatStaticDataMutex);
            auto fbuf = (const float *) buf;
            ahatLUT = std::vector<float>(fbuf, fbuf + (EXPECTED_SIZE / sizeof(float)));
        }

    } else if (name == "R") {  // raw data: AHAT depth + rig2world + interaction
        {
            std::lock_guard<std::mutex> lock(rawDataFrameMutex);
            if (rawDataFrames.size() > MAX_PENDING_FRAMES) {
                // std::cout << "Discard frames" << std::endl;
                return;
            }
        }

        RawDataFrame frame;

        static constexpr size_t TIMESTAMP_SIZE = sizeof(timestamp_t);
        static_assert(sizeof(timestamp_t) == sizeof(int64_t), "timestamp_t not correct");
        static constexpr size_t DEPTH_SIZE = AHAT_WIDTH * AHAT_HEIGHT * sizeof(uint16_t);
        static constexpr size_t RIG2WORLD_SIZE = 16 * sizeof(float);
        static constexpr size_t INTERACTION_SIZE =
                (16 + HandIndexCount * (1 + HandJointIndexCount * (1 + 16)) + 1 + 4 + 4) * sizeof(float);
        static constexpr size_t TOTAL_SIZE = TIMESTAMP_SIZE + DEPTH_SIZE + RIG2WORLD_SIZE + INTERACTION_SIZE;
        if (size != TOTAL_SIZE) {
            std::cerr << "Invalid raw data size size: " << size << ", expecting " << TOTAL_SIZE << std::endl;
            return;
        }

        // Timestamp
        {
            frame.timestamp = *(reinterpret_cast<const timestamp_t *>(buf));
            buf += TIMESTAMP_SIZE;
        }

        // Depth
        {
            auto ubuf = reinterpret_cast<const uint16_t *>(buf);
            frame.depth.assign(ubuf, ubuf + (DEPTH_SIZE / sizeof(uint16_t)));
            buf += DEPTH_SIZE;
        }

        // Rig2world
        {
            auto fbuf = reinterpret_cast<const float *>(buf);
            frame.rig2world = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) fbuf);
            buf += RIG2WORLD_SIZE;
        }
        DirectX::XMMATRIX world2rig = DirectX::XMMatrixInverse(nullptr, frame.rig2world);

        // Interaction
        {
            auto fbuf = reinterpret_cast<const float *>(buf);

            // Head
            frame.headTransformation = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) fbuf);
            fbuf += 16;

            // Hands
            for (auto &hand: frame.hands) {
                hand.tracked = (*fbuf == 1.0f);
                fbuf++;

                for (auto &joint: hand.joints) {
                    joint.tracked = (*fbuf == 1.0f);
                    fbuf++;

                    joint.transformationInWorld = DirectX::XMLoadFloat4x4((const DirectX::XMFLOAT4X4 *) fbuf);
                    fbuf += 16;

                    joint.translationInWorld = XMTransformToTranslate(joint.transformationInWorld);
                    joint.translationInWorld /= DirectX::XMVectorGetW(joint.translationInWorld);
                }

                // Hand tracked + all joints tracked
                hand.strictlyTracked = hand.tracked && countTrackedJoints(hand) == HandJointIndexCount;
            }

            // Eye
            frame.eyeTracked = (*fbuf == 1.0f);
            fbuf++;

            frame.eyeOrigin = DirectX::XMLoadFloat4((const DirectX::XMFLOAT4 *) fbuf);
            fbuf += 4;

            frame.eyeDirection = DirectX::XMLoadFloat4((const DirectX::XMFLOAT4 *) fbuf);
            fbuf += 4;

            buf += INTERACTION_SIZE;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(rawDataFrameMutex);
            rawDataFrames.emplace(std::move(frame));
        }

    } else if (name == "p") {  // point cloud
        std::pair<timestamp_t, PCD> frame;

        /* memcpy(&frame.first, buf, sizeof(timestamp_t));
        buf += sizeof(timestamp_t);
        size -= sizeof(timestamp_t); */
        // TODO: use timestamp from HoloLens
        frame.first = time(nullptr);

        if (size % 12 != 0) {
            std::cerr << "Invalid point cloud size: " << size << ", expecting multiples of 12" << std::endl;
            return;
        }
        size_t count = size / 12;
        frame.second.reserve(count);
        auto fbuf = (const float *) buf;
        for (size_t i = 0; i < count; i++) {
            frame.second.emplace_back(fbuf[0], fbuf[1], fbuf[2]);
            fbuf += 3;
        }

        // Save the data
        {
            std::lock_guard<std::mutex> lock(pcdMutex);
            pcdFrames.emplace(std::move(frame));
        }
    }
}
