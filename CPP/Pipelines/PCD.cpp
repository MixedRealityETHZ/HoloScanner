//
// Created by Zikai Liu on 11/12/22.
//

//#include <DirectXMath.h>

#include <igl/read_triangle_mesh.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/slice.h>

#include <iostream>

#include "TCPDataSource.h"
#include "EigenHelpers.h"
#include "Registrator.h"
#include "DirectXHelpers.h"

TCPDataSource tcpDataSource;

bool callBackPerDraw(igl::opengl::glfw::Viewer &viewer) {
    static timestamp_t pcdTimestamp;
    static PCD pcd;

    static timestamp_t interationTimestamp;
    static RawDataFrame interactionFrame;

    bool updated = false;

    if (tcpDataSource.getNextPCD(pcdTimestamp, pcd)) {
        updated = true;
        /*std::cout << "[PCD] " << pcdTimestamp << std::endl;*/
    }

    if (tcpDataSource.getNextRawDataFrame(interactionFrame)) {
        updated = true;
        /*std::cout << "[INT] " << interationTimestamp
                  << " Left Hand Tracked: " << interactionFrame.hands[Left].tracked
                  << " Right Hand Tracked: " << interactionFrame.hands[Right].tracked
                  << std::endl;*/
    }

    if (updated) {
        viewer.data().clear();

        // PCD
        Eigen::MatrixXd points(pcd.size(), 3);
        for (int i = 0; i < pcd.size(); i++) {
            points.row(i) = pcd[i].cast<double>();
        }
        viewer.data().add_points(points, Eigen::RowVector3d(1, 1, 1));

        // Interaction
        static const Eigen::RowVector3d handColors[HandIndexCount] = {Eigen::RowVector3d(0, 0, 1),
                                                                      Eigen::RowVector3d(1, 0, 0)};
        for (int i = 0; i < HandIndexCount; i++) {
            if (!interactionFrame.hands[i].tracked) continue;

            Eigen::MatrixXd jointPoints((int) HandJointIndexCount, 3);
            for (int j = 0; j < HandJointIndexCount; j++) {
                jointPoints.row(j) = XMVectorToEigenVector3d(
                        XMTransformToTranslate(interactionFrame.hands[i].joints[j].transformationInWorld));
            }

            static const Eigen::MatrixXi edgesBetweenJoints = (Eigen::MatrixXi(24, 2) << 1, 2,
                    2, 3,
                    3, 4,
                    4, 5,

                    1, 6,
                    6, 7,
                    7, 8,
                    8, 9,
                    9, 10,

                    1, 11,
                    11, 12,
                    12, 13,
                    13, 14,
                    14, 15,

                    1, 16,
                    16, 17,
                    17, 18,
                    18, 19,
                    19, 20,

                    1, 21,
                    21, 22,
                    22, 23,
                    23, 24,
                    24, 25).finished();

//            std::cout << jointPoints << std::endl;

            viewer.data().set_edges(jointPoints, edgesBetweenJoints, handColors[i]);
        }

        // Set camera on first frame
        static bool is_first_frame = true;
        if (is_first_frame) {
            viewer.core().align_camera_center(points);
            is_first_frame = false;
        }
    }

    return false;
}

int main() {
    Registrator registrator;

    igl::opengl::glfw::Viewer viewer;
    viewer.callback_pre_draw = callBackPerDraw;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.data().point_size = 2;
    viewer.core().is_animating = true;
    Eigen::Vector4f color(1, 1, 1, 1);
    viewer.core().background_color = color * 0.5f;
    viewer.launch();
    std::cout << "?";
}