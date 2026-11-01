cmake_minimum_required(VERSION 3.0.2)
project(holydrone_identification)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
  std_srvs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS mavros_msgs roscpp std_msgs std_srvs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(identification_fsm src/identification_fsm.cpp)
target_link_libraries(identification_fsm ${catkin_LIBRARIES})

add_executable(brct1order_excite src/brct1order_excite.cpp)
target_link_libraries(brct1order_excite ${catkin_LIBRARIES})

add_executable(gazebo_physics_excite src/gazebo_physics_excite.cpp)
target_link_libraries(gazebo_physics_excite ${catkin_LIBRARIES})

install(TARGETS identification_fsm brct1order_excite gazebo_physics_excite
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/brct1order_process.py
  scripts/gazebo_physics_process.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
