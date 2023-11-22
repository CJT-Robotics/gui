cmake_minimum_required(VERSION 3.0.2)
project(cjtrobotics_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

set(SOURCES src/driver_gui.cpp)
set(IMGUI_SOURCES
  imgui/imgui.cpp
  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/imgui_tables.cpp
  imgui/backends/imgui_impl_glfw.cpp
  imgui/backends/imgui_impl_opengl3.cpp
)
set(STB_SOURCES
  stb/deprecated/stb_image.c
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  stb
)

add_executable(driver_gui ${SOURCES} ${IMGUI_SOURCES} ${STB_SOURCES})
target_include_directories(driver_gui PRIVATE imgui)
target_link_libraries(driver_gui ${catkin_LIBRARIES} glfw zbar OpenGL::GL)

add_executable(operator_gui src/operator_gui.cpp)
target_link_libraries(operator_gui ${catkin_LIBRARIES})
