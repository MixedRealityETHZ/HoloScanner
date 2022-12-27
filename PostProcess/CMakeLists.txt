cmake_minimum_required(VERSION 3.18)
project(postprocess)

##############################################################################################
# CMAKE BY RUNNING: cmake -DOpen3D_ROOT=${HOME}/open3d_install -DCMAKE_BUILD_TYPE=Release .. #
##############################################################################################

find_package(Open3D REQUIRED) # This must be executed before CMAKE_MODULE_PATH gets reset below

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"   ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"     ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"    ON)

find_package(LIBIGL REQUIRED QUIET)

# Add your project files
FILE(GLOB SRCFILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRCFILES})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui Open3D::Open3D)
