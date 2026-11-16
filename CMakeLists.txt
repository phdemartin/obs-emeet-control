cmake_minimum_required(VERSION 3.16)
project(obs-emeet-control VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBOBS REQUIRED libobs)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs objdetect)

add_library(obs-emeet-control MODULE
    src/plugin-main.cpp
)

set_target_properties(obs-emeet-control PROPERTIES PREFIX "")

target_link_libraries(obs-emeet-control PRIVATE
    ${LIBOBS_LIBRARIES}
    ${OpenCV_LIBS}
)

target_include_directories(obs-emeet-control PRIVATE
    ${LIBOBS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(TARGETS obs-emeet-control
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/obs-plugins"
)
