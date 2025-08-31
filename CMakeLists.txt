cmake_minimum_required(VERSION 3.8)
project(image_view_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# global compile options
add_compile_options(-Wall -Wextra)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_executable(${PROJECT_NAME}
  src/image_view_qt.cpp
)
ament_auto_add_library(image_view_qt_lib SHARED
  src/image_view_qt.cpp
)

ament_auto_add_executable(image_view_birds_eye_node
  src/image_view_birds_eye.cpp
)
ament_auto_add_library(image_view_birds_eye_lib SHARED
  src/image_view_birds_eye.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

