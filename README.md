# manipulator-workspace-visualize
visualize manipulator workspace

ws_visualize:
```
colcon build
source install/setup.bash
```

txt_to_pcd:
```
cd txt_to_pcd/
mkdir build && cd build
cmake ..
make
```

1. generate workspace pointcloud
```
ros2 launch ws_visualize ee_sampler_launch.py
```
2. move generated .txt file to txt_to_pcd/build, then execute txt2pcd
```
./txt2pcd
```
3. move generated .pcd file to PointCloudToMesh_Open3D/, then execute main.py
```
python3 main.py
```