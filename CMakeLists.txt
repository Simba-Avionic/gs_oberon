cmake_minimum_required(VERSION 3.8)
project(oberon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gs_interfaces REQUIRED)

include_directories(include/oberon)

set(SOURCES
  src/oberon.cpp
  include/oberon/nau7802/NAU7802.cpp
  include/oberon/Tankowacz.cpp
  include/oberon/Tenso.cpp
)

add_executable(oberon ${SOURCES})
ament_target_dependencies(oberon rclcpp gs_interfaces)
target_link_libraries(oberon
  i2c
)

install(TARGETS oberon
  DESTINATION lib/${PROJECT_NAME})

ament_package()
