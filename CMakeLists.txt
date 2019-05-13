#cmake version
cmake_minimum_required(VERSION 3.5)

#project name
set(PROJECT_BINARY_NAME gamepad_reader)
project(${PROJECT_BINARY_NAME})

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# compatibility with c++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Package finder folder
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Widgets finds its own dependencies
find_package(Qt5Widgets REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIR})

# // ------------------------------------------------------------------------------------ //
# // --------------------------------------- ROS ---------------------------------------- //
# // ------------------------------------------------------------------------------------ //
if(DEFINED CATKIN_DEVEL_PREFIX)

  # This will add a C# macro definition to compile ROS functions used to generate the messages
  add_definitions(-DROSSY_RUNNER)
#  #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––#
#  #      adding all the ROS elements inside this IF conditional      #
#  #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––#
#  ## Find catkin macros and libraries
#  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
#  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    genmsg
    message_generation
    roscpp
    std_msgs
  )

  ## Generate messages in the 'msg' folder
  add_message_files(FILES
    gamepads.msg
    logitech_g920.msg
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(DEPENDENCIES std_msgs)

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if you package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
  )

  #ros directory
  set(ROS_INC_DIR /opt/ros/kinetic)

  #set include directories
  set(INC_DIRS
    include
    ${ROS_INC_DIR}/include
    ${catkin_INCLUDE_DIRS}
  )

  #set lib directories
  set(LIB_DIRS
    ${ROS_INC_DIR}/lib
  )

  #include directories
  include_directories(${INC_DIRS})
  link_directories(${LIB_DIRS})

endif(DEFINED CATKIN_DEVEL_PREFIX)
# // ------------------------------------------------------------------------------------ //
# // ------------------------------------- END ROS -------------------------------------- //
# // ------------------------------------------------------------------------------------ //

#source files
set(CPP_FILES
  src/console.cpp
  src/main.cpp
  src/GamepadReader.cpp
)

#header files
set(HPP_FILES
  include/gamepad_reader/console.h
  include/gamepad_reader/definitions.h
  include/gamepad_reader/GamepadReader.h
)

#this creates MOC files for classes that include the Q_OBJECT macro, if 'vtable' error appears then you should include the headers in here
set(WRAP_CPP_FILES
  include/gamepad_reader/console.h
  include/gamepad_reader/GamepadReader.h
)

set(ROS_MESSAGES
  msg/gamepads.msg
  msg/logitech_g920.msg
)

include_directories(include)

#including the GUI forms
set(WRAP_UI_FILES ui/console.ui )
#including the resource files
set(WRAP_RS_FILES resources/images.qrc )

#qt5 requires wrapping of several parts, in this case are: MOC files, forms and resources
qt5_wrap_cpp(PROJECT_HEADERS_MOC ${WRAP_CPP_FILES})
qt5_wrap_ui(PROJECT_FORMS_HEADERS ${WRAP_UI_FILES})
qt5_add_resources(PROJECT_RESOURCES ${WRAP_RS_FILES})

# // ------------------------------------------------------------------------------------ //
# // ----------------------------- COPYING THE CONTROL MAP ------------------------------ //
# // ------------------------------------------------------------------------------------ //

file(COPY ${PROJECT_SOURCE_DIR}/control_maps DESTINATION ${CMAKE_BINARY_DIR})

# // ------------------------------------------------------------------------------------ //
# // --------------------------- END COPYING THE CONTROL MAP ---------------------------- //
# // ------------------------------------------------------------------------------------ //

#add every file to the executable
add_executable(${PROJECT_BINARY_NAME}
  ${CPP_FILES}
  ${HPP_FILES}
  ${PROJECT_HEADERS_MOC}
  ${PROJECT_FORMS_HEADERS}
  ${PROJECT_RESOURCES}
  ${ROS_MESSAGES}
)

#indicates which libraries to use in the executable
target_link_libraries(${PROJECT_BINARY_NAME}
  Qt5::Widgets
  ${SDL2_LIBRARY}
  ${catkin_LIBRARIES}
)
