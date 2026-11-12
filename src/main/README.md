# 注意：
为了方便调试，写了两个main文件：main_pc.cpp&main_cam.cpp,一个在电脑端测试，一个在相机上测试
# 如何灵活切换两个main.cpp:
在 CMakeLists.txt 中灵活切换:
打开CMakeLists.txt找到add_executable
# 在电脑上测试时
add_executable(armor_pc 
    main_pc.cpp 
    vision_core.cpp 
    comm/uart.cpp
)
# 当你要交叉编译并传到相机上时（MaixCam端）：
add_executable(armor_cam 
    main_cam.cpp 
    vision_core.cpp 
    comm/uart.cpp
)
