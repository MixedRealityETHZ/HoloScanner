# Post-Processing Pipeline

## Requirements
* Open3D

## Run
```
mkdir build && cd build
cmake -DOpen3D_ROOT=<path_to_open3d_install> -DCMAKE_BUILD_TYPE=Release ..
make
./postprocess <path_to_xyz_file>
```
For example, `path_to_open3d_install` could be `${HOME}/open3d_install ` and `path_to_xyz_file` could be `../finaldenoisedboxdb.xyz` 