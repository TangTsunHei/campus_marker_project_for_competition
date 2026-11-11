# comm 文件夹为 通信模块
src/main/
 ├── build/             # 编译输出目录
 ├── CMakeLists.txt     # 编译配置文件
 ├── main.cpp           # 主程序（视频流读取、调用检测算法）
 ├── calibrate.cpp      # 标定程序
 ├── comm/    <---------# 【新建】通信模块
 │    ├── protocol.hpp  # 协议定义
 │    ├── uart.hpp      # 串口类的头文件（声明发送、接收、初始化接口）
 │    └── uart.cpp      # 串口类的实现文件（Linux下的 termios 底层操作）

 # 串口提供的数据
 目标值为 0：
 最理想的状态，是让车头正对着兑换框正中心，此时 X = 0，Y = 0，且 Z = 预设的安全投放距离