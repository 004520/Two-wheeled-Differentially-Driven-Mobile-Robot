cmake_minimum_required(VERSION 3.8)
project(mycar_servo_qt_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

set(SOURCES
  include/mycar_servo_qt_gui/servo_publisher.hpp
  include/mycar_servo_qt_gui/main_window.hpp
  src/main.cpp
  src/servo_publisher.cpp
  src/main_window.cpp
)

add_executable(mycar_servo_qt_gui ${SOURCES})
target_include_directories(mycar_servo_qt_gui PRIVATE include)
target_link_libraries(mycar_servo_qt_gui Qt5::Widgets Qt5::Core)
ament_target_dependencies(mycar_servo_qt_gui
  rclcpp
  geometry_msgs
)

install(TARGETS mycar_servo_qt_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
