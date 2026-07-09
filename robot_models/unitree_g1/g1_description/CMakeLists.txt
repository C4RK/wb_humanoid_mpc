cmake_minimum_required(VERSION 3.5)#编译环境
project(g1_description)#项目名称

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)#寻找ros2构建工具

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)#这里进行资源搬运与注册，把文件夹复制到目录下

ament_package()
