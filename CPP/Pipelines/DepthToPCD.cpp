//
// Created by Zikai Liu on 11/12/22.
//

#include <igl/read_triangle_mesh.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include <iostream>

#include "TCPDataSource.h"

#include "DepthProcessor.h"
#include "DirectXHelpers.h"
#include "EigenHelpers.h"
#include "PCDDataTypes.h"
#include "Registrator.h"

class DepthProcessorWrapper : public DepthProcessor, public PCDSource {
public:

    DepthProcessorWrapper(const DirectX::XMMATRIX &extrinsics, const float *lut)
            : DepthProcessor(extrinsics, lut) {}

    bool getNextPCD(timestamp_t &timestamp, PCD &pcd) override {
        PCDRaw pcdRaw;
        if (DepthProcessor::getNextPCDRaw(timestamp, pcdRaw)) {
            assert(pcdRaw.size() % 3 == 0 && "PCDRaw does not contain not flatten 3D vectors");
            pcd.clear();
            pcd.reserve(pcdRaw.size() / 3);
            for (size_t i = 0; i < pcdRaw.size() / 3; i++) {
                // NOTICE: swapping x and y and all negated
                pcd.emplace_back(pcdRaw[i * 3], pcdRaw[i * 3 + 1], pcdRaw[i * 3 + 2]);
            }
            return true;
        } else {
            return false;
        }
    }
};

const Eigen::RowVector3d POINT_COLOR[2] = {
        Eigen::RowVector3d(1, 0.5, 0),  // failed
        Eigen::RowVector3d(1, 1, 1),    // succeeded
};

bool shouldExit = false;
// Even without atomic, resetting more than once (very unlikely) is also not a problem
bool depthProcessorShouldReset = false;
bool registratorShouldReset = false;
bool viewerShouldReset = false;

TCPDataSource tcpDataSource([] {
    // Stop callback
    depthProcessorShouldReset = true;
    registratorShouldReset = true;
    viewerShouldReset = true;
});

std::unique_ptr<DepthProcessorWrapper> depthProcessor;

Registrator registrator;

// Data exchanged between threads

enum DisplayMode { CURRENT_FRAME_PCD, RECONSTRUCTED_PCD };
DisplayMode displayMode = RECONSTRUCTED_PCD;  // not well protected against multithread, reader should handle carefully

std::mutex handDebugFrameLock;
bool handDebugFrameValid = false;
HandDebugFrame handDebugFrame;
bool handDebugFrameUpdated = false;

std::mutex displayPCDLock;
Eigen::MatrixXd displayPCD;
Eigen::RowVector3d displayPCDColor;
bool displayPCDUpdated = false;

std::thread depthToPCDThread([] {
    while (!shouldExit) {
        if (depthProcessorShouldReset) {
            depthProcessor = nullptr;
            depthProcessorShouldReset = false;
        }

        if (!depthProcessor) {
            DirectX::XMMATRIX ahatExtrinsics;
            std::vector<float> ahatLUT;
            if (!tcpDataSource.getAHATExtrinsics(ahatExtrinsics)) continue;
            if (!tcpDataSource.getAHATDepthLUT(ahatLUT)) continue;
            depthProcessor = std::make_unique<DepthProcessorWrapper>(ahatExtrinsics, ahatLUT.data());
            // std::cout << "Create DepthProcessor with AHAT extrinsics and LUT" << std::endl;
        }

        RawDataFrame rawDataFrame;
        if (tcpDataSource.getNextRawDataFrame(rawDataFrame)) {
#if 0
            std::cout << "[Raw] " << rawDataFrame.timestamp << "    lostTracking = " << lostTracking << std::endl;
#endif
            depthProcessor->update(rawDataFrame);
        }
    }
});

/**
 * Send reconstructed PCD and set shared variables to update the viewer
 * @param pcdInMatrix  NOTICE: will be std::moved
 * @param color
 */
void setDisplayPCD(Eigen::MatrixXd &pcdInMatrix, const Eigen::RowVector3d &color) {
    // Give the PCD to the viewer (safe to std::move since we don't need it anymore)
    {
        std::lock_guard _(displayPCDLock);
        displayPCD = std::move(pcdInMatrix);
        displayPCDColor = color;
        displayPCDUpdated = true;
    }
}

std::thread registrationThread([] {
    constexpr int WARM_UP_FRAME_COUNT = 20;

    int warmUPFrameRemaining = WARM_UP_FRAME_COUNT;

    int frameCounter = 0;
    auto lastStatTime = std::chrono::steady_clock::now();

    while (!shouldExit) {
        if (registratorShouldReset) {
            std::cout << "========== RECEIVED STOP SIGNAL ===========" << std::endl;

            try {
                registrator.saveReconstructedMesh();  // will change PCD inside
            } catch (...) {
                std::cerr << "Error saving results" << std::endl;
            }

            Eigen::MatrixXd objectPCD;
            if (registrator.getReconstructedPCDInEigenFormat(objectPCD)) {
                std::cout << "[FinalPCD] " << objectPCD.size() << std::endl;
                tcpDataSource.sendReconstructedPCD(Eigen::RowVector3d(0, 0.8, 1), objectPCD);
                if (displayMode == RECONSTRUCTED_PCD) {
                    setDisplayPCD(objectPCD, Eigen::RowVector3d(0, 0.8, 1));
                    // NOTICE: objectPCD is moved
                }
            } else {
                std::cout << "No reconstructed PCD available" << std::endl;
            }

            registrator.reset();
            registratorShouldReset = false;
        }

        if (!depthProcessor) continue;

        auto now = std::chrono::steady_clock::now();
        auto escapedMS = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStatTime).count();
        if (escapedMS >= 1000) {
            std::cout << "Frame rate: " << frameCounter << std::endl;
            frameCounter = 0;
            lastStatTime = now;
        }

        timestamp_t pcdTimestamp;
        PCD pcd;
        if (depthProcessor->getNextPCD(pcdTimestamp, pcd)) {
#if 1
            std::cout << "[PCD] " << pcd.size() << std::endl;
#endif

            // Get hand debug frame for visualizer
            {
                std::lock_guard _(handDebugFrameLock);
                timestamp_t handDebugFrameTimestamp;
                handDebugFrameValid = depthProcessor->getNextHandDebugFrame(handDebugFrameTimestamp, handDebugFrame);
                handDebugFrameUpdated = true;
            }

            if (warmUPFrameRemaining) {
                warmUPFrameRemaining--;
                continue;
            }

#ifdef USE_DBSCAN
            bool succeeded = registrator.mergePCD(pcd, depthProcessor->handMesh);
#else
            bool succeeded = registrator.mergePCD(pcd);
#endif
            frameCounter++;

            // Continue to send existing data regardless succeeded or not, as long as there are existing reconstruction
            Eigen::MatrixXd objectPCD;
            if (registrator.getReconstructedPCDInEigenFormat(objectPCD)) {
#if 1
                std::cout << "[ReconstructedPCD] " << objectPCD.rows() << "  succeeded = " << succeeded << std::endl;
#endif
                tcpDataSource.sendReconstructedPCD(POINT_COLOR[succeeded], objectPCD);

                if (displayMode == RECONSTRUCTED_PCD) {
                    setDisplayPCD(objectPCD, POINT_COLOR[succeeded]);
                    // NOTICE: objectPCD is moved
                }
            }

            if (displayMode == CURRENT_FRAME_PCD) {
                Eigen::MatrixXd points(pcd.size(), 3);
                for (int i = 0; i < pcd.size(); i++) {
                    points.row(i) = pcd[i];
                }
                setDisplayPCD(points, Eigen::RowVector3d(1, 1, 1));
                // NOTICE: pcd is moved, so this must be placed at last
            }
        }

    }
});

bool callBackPerDraw(igl::opengl::glfw::Viewer &viewer) {
    static bool viewHasSet = false;
    if (viewerShouldReset) {
        viewHasSet = false;
        viewerShouldReset = false;
    }

    bool redraw = false;

    // Static to keep last data if only one of them is updated
    static HandDebugFrame hand;
    static Eigen::MatrixXd pcd;
    static Eigen::RowVector3d pcdColor;

    {
        std::lock_guard _(handDebugFrameLock);
        if (handDebugFrameUpdated) {
            hand = std::move(handDebugFrame);  // safe to move since it's only used to pass in here
            handDebugFrameUpdated = false;
            redraw = true;
        }
    }

    {
        std::lock_guard _(displayPCDLock);
        if (displayPCDUpdated) {
            pcd = std::move(displayPCD);  // safe to move since it's only used to pass in here
            pcdColor = std::move(displayPCDColor);
            displayPCDUpdated = false;
            redraw = true;
        }
    }

    if (redraw) {
        viewer.data().clear();

        // Reconstructed PCD
        if (pcd.rows() > 0) {
            viewer.data().add_points(pcd, pcdColor);

            // Set camera on first frame
            if (!viewHasSet) {
                viewer.core().align_camera_center(pcd);
                viewHasSet = true;
            }
        }

        // Hands
        int lhvSize = static_cast<int>(hand.lhMesh.size());
        int lhiSize = static_cast<int>(hand.lhIndices.size());
        int rhvSize = static_cast<int>(hand.rhMesh.size());
        int rhiSize = static_cast<int>(hand.rhIndices.size());
        Eigen::MatrixXd jointPoints(lhvSize + rhvSize, 3);
        Eigen::MatrixXi jointIndices(lhiSize + rhiSize, 2);
        Eigen::MatrixXd colors(lhiSize + rhiSize, 3);

        for (int j = 0; j < lhvSize; j++) {
            jointPoints.row(j) = XMVectorToEigenVector3d(hand.lhMesh[j]);
        }
        for (int j = 0; j < rhvSize; j++) {
            jointPoints.row(j + lhvSize) = XMVectorToEigenVector3d(hand.rhMesh[j]);
        }

        for (int j = 0; j < lhiSize; j++) {
            jointIndices.row(j) << hand.lhIndices[j][0], hand.lhIndices[j][1];
            colors.row(j) = Eigen::RowVector3d(0, 1, 0);
        }
        for (int j = 0; j < rhiSize; j++) {
            jointIndices.row(j + lhiSize) << hand.rhIndices[j][0] + lhvSize, hand.rhIndices[j][1] + lhvSize;
            colors.row(j + lhiSize) = Eigen::RowVector3d(1, 0, 0);
        }

        viewer.data().set_edges(jointPoints, jointIndices, colors);
    }

    return false;
}

int main() {
    std::ios::sync_with_stdio(false);

    igl::opengl::glfw::Viewer viewer;

    // Menu
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);
    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();  // draw parent menu content

        if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
            if(ImGui::Combo("Display mode", (int*)(&displayMode), "CURRENT_FRAME_PCD\0RECONSTRUCTED_PCD\0\0")) {
                // nothing
            }
        }
    };

    viewer.callback_pre_draw = callBackPerDraw;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.data().point_size = 2;
    viewer.data().line_width = 10;
    viewer.core().is_animating = true;
    Eigen::Vector4f color(1, 1, 1, 1);
    viewer.core().background_color = color * 0.0f;
    viewer.launch();  // blocking

    shouldExit = true;
    depthToPCDThread.join();
    registrationThread.join();
}