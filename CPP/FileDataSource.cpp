//
// Created by Zikai Liu on 11/15/22.
//

#include "FileDataSource.h"

#include <utility>
#include <fstream>
#include <iostream>

FileDataSource::FileDataSource(std::filesystem::path folder_) : folder(std::move(folder_)) {
    std::cout << "Loading data from " << folder << std::endl;

    readAHATExtrinsics();
    readAHATLUT();

    std::vector<DirectX::XMMATRIX> rig2Worlds;
    std::vector<timestamp_t> rig2WorldTimestamps;
    readRig2World(rig2Worlds, rig2WorldTimestamps);

    auto pgmFiles = enumeratePGMFiles();
    for (const auto &pgmFile : pgmFiles) {
        AHATFrame frame;
        readPGM(pgmFile, frame.timestamp, frame.depth);
        frame.rig2world = rig2Worlds[findClosestTimestampIndex(frame.timestamp, rig2WorldTimestamps)];
        ahatFrames.emplace_back(std::move(frame));
    }

    std::cout << "Data loaded" << std::endl;
}

void FileDataSource::readAHATExtrinsics() {
    std::ifstream inFile(folder / "Depth AHaT_extrinsics.txt");
    std::string inputLine;
    DirectX::XMFLOAT4X4 ext;
    int j = 0, i = 0;
    while (std::getline(inFile, inputLine, ',')) {
        float f = std::stof(inputLine);
        ext.m[j][i] = f;
        i++;
        if (i == 4) {
            i = 0;
            j++;
        }
    }

    // Notice: transposed
    ahatExtrinsics = DirectX::XMMatrixTranspose(DirectX::XMLoadFloat4x4(&ext));
}

void FileDataSource::readAHATLUT() {
    std::ifstream inFile(folder / "Depth AHaT_lut.bin", std::ios::binary);
    ahatLUT.resize(3 * AHAT_WIDTH * AHAT_HEIGHT);
    bool success = !inFile.read(reinterpret_cast<char *>(ahatLUT.data()),
                                3 * AHAT_WIDTH * AHAT_HEIGHT * sizeof(float)).fail();
    assert(success && "Invalid LUT file");
}

void FileDataSource::readRig2World(std::vector<DirectX::XMMATRIX> &rig2Worlds, std::vector<timestamp_t> &timestamps) {
    std::ifstream inFile(folder / "Depth AHaT_rig2world.txt");
    std::string inputLine;
    while (getline(inFile, inputLine)) {
        DirectX::XMFLOAT4X4 ext;
        int j = 0, i = 0;
        std::stringstream line(inputLine);
        std::string timestamp;
        std::string nextVal;
        getline(line, timestamp, ',');
        timestamps.push_back(stol(timestamp));
        while (getline(line, nextVal, ',')) {
            float f = std::stof(nextVal);
            ext.m[j][i] = f;
            i++;
            if (i == 4) {
                i = 0;
                j++;
            }
        }
        // Notice: transposed
        rig2Worlds.push_back(DirectX::XMMatrixTranspose(DirectX::XMLoadFloat4x4(&ext)));
    }
}

std::vector<std::string> FileDataSource::enumeratePGMFiles() {
    std::vector<std::string> paths;
    for (const auto &entry: std::filesystem::directory_iterator(folder / "Depth AHaT")) {
        std::string path = entry.path();
        if (path[path.length() - 7] == '_')  // discard _ab.pgm files
            continue;
        paths.push_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

void FileDataSource::readPGM(const std::filesystem::path &filename, timestamp_t &timestamp, AHATDepth &data) {

    std::stringstream ss;
    std::ifstream infile(filename, std::ios::binary);

    std::string inputLine;
    std::getline(infile, inputLine); // read the first line : P5
    assert(inputLine == "P5" && "PGM version error");

    ss << infile.rdbuf(); // read the third line : width and height
    int rowCount = 0, colCount = 0;
    ss >> colCount >> rowCount;
    assert(rowCount == AHAT_WIDTH && rowCount == AHAT_HEIGHT);

    int maxVal;  // maximum intensity value
    ss >> maxVal;
    assert(maxVal == 65535);
    ss.ignore();

    timestamp = std::stoull(filename.filename());

    data.clear();
    data.reserve(AHAT_WIDTH * AHAT_HEIGHT);
    for (int row = 0; row < rowCount; row++) {
        for (int col = 0; col < colCount; col++) {
            uint16_t pixel;
            ss.read((char *) &pixel, sizeof(uint16_t));
            data.emplace_back(swapEndian(pixel));
        }
    }
}

int FileDataSource::findClosestTimestampIndex(timestamp_t ts, std::vector<timestamp_t> timestamps) {
    // TODO: do binary search
    int64_t shortestTime = std::numeric_limits<int64_t>::max();
    int index = 0;
    for (int i = 0; i < timestamps.size(); i++) {
        int64_t dt = std::abs((int64_t) timestamps[i] - (int64_t) ts);
        if (dt < shortestTime) {
            shortestTime = dt;
            index = i;
        }
    }
    return index;
}

bool FileDataSource::getAHATExtrinsics(DirectX::XMMATRIX &extrinsics)  {
    extrinsics = ahatExtrinsics;
    return true;
}

bool FileDataSource::getAHATDepthLUT(AHATLUT &lut)  {
    lut = ahatLUT;
    return true;
}

bool FileDataSource::getNextAHATFrame(timestamp_t &timestamp, AHATDepth &depth, DirectX::XMMATRIX &rig2world) {
    if (ahatFrameIndex >= ahatFrames.size()) return false;
    AHATFrame &frame = ahatFrames[ahatFrameIndex];
    timestamp = frame.timestamp;
    depth = frame.depth;
    rig2world = frame.rig2world;
    ahatFrameIndex++;
    return true;
}

void FileDataSource::resetFromStart(bool realTimeSimulation) {
    ahatFrameIndex = 0;
}
