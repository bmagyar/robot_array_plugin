cmake_minimum_required(VERSION 2.8.3)
project(robot_array_plugin)
add_compile_options(-std=c++11)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS 
  rviz
  geometry_msgs
  urdf
)

find_package(urdfdom_headers REQUIRED)

catkin_package(
    DEPENDS 
      rviz
    CATKIN_DEPENDS 
      geometry_msgs
      urdf
    LIBRARIES 
      ${PROJECT_NAME}
)

include_directories(
  src
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set(SOURCE_FILES src/ambient_sound_display.cpp src/ambient_sound_visual.cpp)

set(HEADER_FILES
  src/robot_array_display.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

set(SOURCE_FILES
  src/robot_array_display.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${UIC_FILES})
if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# install(DIRECTORY src/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
