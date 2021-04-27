project(CarMaker)
cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#find_package(Qt5Widgets REQUIRED)
#include_directories(${Qt5Widgets_INCLUDE_DIRS})
#add_definitions(${Qt5Widgets_DEFINITIONS})

#compiler and linker flags
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++  ${Qt5Widgets_LIBRARIES}")
set(CMAKE_CXX_FLAGS "-O3 -march=i686 -DNDEBUG -DWIN32 -DCM_NUMVER=50002 -DMYMODELS -D__USE_MINGW_ANSI_STDIO -std=c++11 ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SDL2 REQUIRED)

#sources
set(SOURCES
#    CM_Main.cpp
#    CM_Vehicle.cpp
#    User.cpp
#    app_tmp.cpp
#    ADT/ADTMain.cpp
#    ADT/CMUserOutput.cpp
    GamepadReader.cpp
)

set(HPP_FILES
#  ADT/ADTMain.h
  GamepadReader.h
  definitions.h
)

#carmaker
#remember to change the carmake's directory
set(CARMAKER_DIR C:/IPG/hil/win32-5.0.3)
set(CARMAKER_LIB_DIR ${CARMAKER_DIR}/lib)
set(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)
set(CARMAKER_MATLIB_DIR ${CARMAKER_DIR}/Matlab/R2015a)

#boost
#remember to change boost's directory
set(BOOST_INC_DIR C:/boost_1_60_0)

set(INC_DIRS
${CARMAKER_DIR}
${CARMAKER_INC_DIR}
${SDL2_INCLUDE_DIR}
)

set(LIB_DIRS
${CARMAKER_LIB_DIR}
${CARMAKER_MATLIB_DIR}
)

#include directories
include_directories(${INC_DIRS})
link_directories(${LIB_DIRS})

#build settings
add_executable(CarMaker.win32.exe
  ${SOURCES}
  ${HPP_FILES}
)
TARGET_LINK_LIBRARIES(CarMaker.win32.exe car carmaker ipgdriver ipgroad ipgtire tametire matsupp-win32 z-win32 m ws2_32 psapi
  ${SDL2_LIBRARY}
  #Qt5::Widgets
)
