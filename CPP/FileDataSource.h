//
// Created by Zikai Liu on 11/15/22.
//

#ifndef HOLOSCANNER_FILEDATASOURCE_H
#define HOLOSCANNER_FILEDATASOURCE_H

#include "RawDataTypes.h"
#include "InteractionDataTypes.h"
#include "PCDDataTypes.h"
#include <filesystem>
#include <queue>

class FileDataSource : public AHATSource, public InteractionSource {
public:

    explicit FileDataSource(std::filesystem::path folder);

    void resetFromStart(bool realTimeSimulation);

    bool getAHATExtrinsics(DirectX::XMMATRIX &extrinsics) override;

    bool getAHATDepthLUT(AHATLUT &lut) override;

    bool getNextAHATFrame(timestamp_t &timestamp, AHATDepth &depth, DirectX::XMMATRIX &rig2world) override;

    bool getNextInteractionFrame(timestamp_t &timestamp, InteractionFrame &frame) override { return false; }

private:

    std::filesystem::path folder;

    DirectX::XMMATRIX ahatExtrinsics;
    std::vector<float> ahatLUT;

    std::vector<AHATFrame> ahatFrames;
    int ahatFrameIndex = 0;

    void readAHATExtrinsics();

    void readAHATLUT();

    void readRig2World(std::vector<DirectX::XMMATRIX> &rig2Worlds, std::vector<timestamp_t> &timestamps);

    std::vector<std::string> enumeratePGMFiles();

    void readPGM(const std::filesystem::path &filename, timestamp_t &timestamp, AHATDepth &data);

    static int findClosestTimestampIndex(timestamp_t ts, std::vector<timestamp_t> timestamps);

    static uint16_t swapEndian(uint16_t x) { return (x >> 8) | (x << 8); }
};


#endif //HOLOSCANNER_FILEDATASOURCE_H
