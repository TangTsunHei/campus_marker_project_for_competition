# README.md（包含从配置、编译到运行的完整命令）

# 环境配置：

本次作业在Linux环境中测试并通过：
Ubuntu 24.04 （在wsl ubuntu24.04上已验证）
GCC 13.3.0  （在wsl ubuntu24.04上已验证）
CMake >= 3.22  （在wsl ubuntu24.04上已验证，为3.28.3）
OpenCV 4.6.0  （在wsl ubuntu24.04上已验证）
C++17


# 依赖安装方法:
在全新环境中，请先执行以下命令安装必要依赖：
bash:
sudo apt update
sudo apt install build-essential cmake libopencv-dev

# 相机标定：
用maixcam pro拍摄了一段1:42的标定板视频，并用该视频完成了相机标定任务（获得相机内参和畸变系数）
这些获得的参数将被定死，一直用到校内赛结束，！！！注意：相机标定后不能改变相机分辨率和焦距，否则标定参数将失效

