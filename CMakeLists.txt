cmake_minimum_required(VERSION 3.10)

project(depixelization VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED true)

find_package(OpenCV REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(glm CONFIG REQUIRED)

add_subdirectory(external/glad/)

include_directories(${OpenCV_INCLUDE_DIRS})

# imgui
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# source files
file(GLOB MAIN_SOURCE src/main.cpp)
file(GLOB OTHER_SOURCES src/*.cpp)

# combine all source files
set(SOURCES
  ${IMGUI_SOURCES}
  ${OTHER_SOURCES}
  ${MAIN_SOURCE}
)

add_executable(depixelization ${SOURCES})

target_sources(depixelization PRIVATE ${IMGUI_SOURCES})

target_link_libraries(depixelization glfw glm::glm ${OpenCV_LIBS} glad)

target_include_directories(depixelization PUBLIC
  "${PROJECT_SOURCE_DIR}/include"
  "${IMGUI_DIR}"
  "${IMGUI_DIR}/backends"
)

add_custom_command(TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR}
)