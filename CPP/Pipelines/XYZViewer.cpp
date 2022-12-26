//
// Created by Zikai Liu on 12/14/22.
//

#include <igl/read_triangle_mesh.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include "XYZLoader.h"

#include <iostream>

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

        // XYZ
        if (ImGui::CollapsingHeader("XYZ", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float w = ImGui::GetContentRegionAvail().x;
            float p = ImGui::GetStyle().FramePadding.x;
            if (ImGui::Button("Load##XYZ", ImVec2((w-p), 0)))
            {
                std::string fname = igl::file_dialog_open();
                if (fname.length() == 0)
                    return;

                auto mat = loadPointCloudFromXYZ(fname);

                viewer.data().clear();
                viewer.data().add_points(mat, Eigen::RowVector3d(0, 0.8, 1));
            }
        }
    };

//    viewer.callback_pre_draw = callBackPerDraw;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.data().point_size = 2;
    viewer.data().line_width = 10;
    viewer.core().is_animating = true;
    viewer.core().background_color = Eigen::Vector4f(0.349, 0.349, 0.349, 1);
    viewer.launch();  // blocking

    return 0;
}