cmake_minimum_required(VERSION 3.5)

project(cube VERSION 1.0 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -pedantic -Wshadow -Wold-style-cast -Wunused -Wsign-conversion -Wuseless-cast -Wdouble-promotion -Wduplicated-cond -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-conversion -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wformat=2 -g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wextra -pedantic -Wshadow -Wold-style-cast -Wunused -O3 -DNDEBUG ")

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(SOURCES
    main.cpp
    mainwidget.cpp
    geometryengine.cpp
    mpu6050.cpp
)

set(HEADERS
    mainwidget.h
    geometryengine.h
    mpu6050.hpp
)

# set(UIS
#     ui/mainwindow.ui
#     ui/list_item.ui
# )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_sources(${PROJECT_NAME} PRIVATE
    shaders.qrc
    textures.qrc
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Qt5::Widgets
)