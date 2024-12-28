编译，安装
```shell
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
若没有安装CUDA，则为
```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
选择特定包编译时，为
```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select xxx