#cmake version
cmake_minimum_required(VERSION 2.8.12)

#project name
project(gamepad-CMAKE)
set(PROJECT_BINARY_NAME gamepad-CMAKE)

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#compatibility with C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Widgets finds its own dependencies
find_package(Qt5Widgets REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIR})

#source files
set(CPP_FILES
  src/console.cpp
  src/main.cpp
  src/GamepadReader.cpp
)

#header files
set(HPP_FILES
  includes/console.h
  includes/GamepadReader.h
)

#this creates MOC files for classes that include the Q_OBJECT macro, if 'vtable' error appears then you should include the headers in here
set(WRAP_CPP_FILES
  includes/console.h
  includes/GamepadReader.h
)
#including the GUI forms
set(WRAP_UI_FILES forms/console.ui)
#including the resource files
set(WRAP_RS_FILES
 resources/images.qrc
 resources/gamepad_maps.qrc
)

#qt5 requires wrapping of several parts, in this case are: MOC files, forms and resources
qt5_wrap_cpp(PROJECT_HEADERS_MOC ${WRAP_CPP_FILES})
qt5_wrap_ui(PROJECT_FORMS_HEADERS ${WRAP_UI_FILES})
qt5_add_resources(PROJECT_RESOURCES ${WRAP_RS_FILES})

#add every file to the executable
add_executable(${PROJECT_BINARY_NAME}
  ${CPP_FILES}
  ${HPP_FILES}
  ${PROJECT_HEADERS_MOC}
  ${PROJECT_FORMS_HEADERS}
  ${PROJECT_RESOURCES}
)

#indicates which libraries to use in the executable
target_link_libraries(${PROJECT_BINARY_NAME}
  Qt5::Widgets
  ${SDL2_LIBRARY}
)
