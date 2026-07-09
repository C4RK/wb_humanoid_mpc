cmake_minimum_required(VERSION 3.5)
project(g1_centroidal_mpc)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif ()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)#自动生成.json文件

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)#ROS2的c++核心通信库
find_package(humanoid_centroidal_mpc_ros2 REQUIRED)#自己写的ROS2封装包，用来处理OCS2框架与ROS2之间的数据转换

find_package(GTest REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

# find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)#刚体动力学的开源库

find_package(Eigen3 3.3 REQUIRED NO_MODULE)#C++线性代数库


###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 20)#使用c++20进行编译

set(FLAGS ${OCS2_CXX_FLAGS} -Wno-invalid-partial-specialization)

include_directories(#把外部库的头文件路径打包告诉编译器
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

set(CMAKE_CXX_STANDARD 20)

set(FLAGS ${OCS2_CXX_FLAGS} -Wno-invalid-partial-specialization)

set(dependencies
  ament_cmake
  ament_index_cpp
  rclcpp
  humanoid_centroidal_mpc_ros2
)
#生成可执行文件test_pinocchio_model，读取/testPinocchioModel.cpp
add_executable(test_pinocchio_model
  test/testPinocchioModel.cpp
)
ament_target_dependencies(test_pinocchio_model
  ${dependencies}
)
target_link_libraries(test_pinocchio_model
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(test_pinocchio_model PUBLIC ${FLAGS})

#############
## Install ##
#############

install(TARGETS 
  test_pinocchio_model 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
