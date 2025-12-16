cmake_minimum_required(VERSION 3.8)
project(dh_gripper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装目录配置
install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()