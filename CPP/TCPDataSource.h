//
// Created by Zikai Liu on 11/12/22.
//

#ifndef HOLOSCANNER_TCPDATASOURCE_H
#define HOLOSCANNER_TCPDATASOURCE_H

#include "RawDataTypes.h"
#include "PCDDataTypes.h"
#include "TerminalSocket.h"
#include <queue>

class TCPDataSource : public RawDataSource, public PCDSource {
public:

    // NOTICE: stopCallBack will be called from another thread
    explicit TCPDataSource(std::function<void()> stopCallBack);

    ~TCPDataSource();

    bool getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) override;

    bool getAHATDepthLUT(AHATLUT &lut) override;

    bool getNextRawDataFrame(RawDataFrame &frame) override;

    bool getNextPCD(timestamp_t &timestamp, PCD &pcd) override;

    bool sendReconstructedPCD(const Eigen::RowVector3d &pointColor, const Eigen::MatrixXd &pcd);

private:

    static constexpr int PORT = 9090;
    static constexpr int MAX_PENDING_FRAMES = 3;

    boost::asio::io_context tcpIOContext;
    std::thread *tcpIOThread = nullptr;

    std::function<void()> stopCallBack;

    TerminalSocketServer socketServer;

    void handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size);

    bool ahatExtrinsicsValid = false;
    DirectX::XMMATRIX ahatExtrinsics;
    std::vector<float> ahatLUT;
    std::mutex ahatStaticDataMutex;

    std::queue<RawDataFrame> rawDataFrames;
    std::mutex rawDataFrameMutex;

    std::queue<std::pair<timestamp_t, PCD>> pcdFrames;
    std::mutex pcdMutex;
};


#endif //HOLOSCANNER_TCPDATASOURCE_H
