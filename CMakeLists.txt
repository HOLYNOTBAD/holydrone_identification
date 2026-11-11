# holydrone_identification 的 Catkin 构建、安装与测试注册。
cmake_minimum_required(VERSION 3.0.2)
project(holydrone_identification)

# ============================ 构建依赖 ============================
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  mavros_msgs
  roscpp
  std_msgs
  std_srvs
)

# 使用 setup.py 安装本包 Python 模块，供离线处理脚本导入。
catkin_python_setup()
# Noetic 的独立模块 relay 在隔离字典中执行，不能导出函数；开发空间直接复制源码。
configure_file(scripts/brct1order_regression.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/brct1order_regression.py COPYONLY)
configure_file(scripts/brct2order_regression.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/brct2order_regression.py COPYONLY)
configure_file(scripts/brct3order_regression.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/brct3order_regression.py COPYONLY)
# 导出本包供其他 Catkin 包使用的接口与依赖。
catkin_package(INCLUDE_DIRS include DEPENDS EIGEN3 CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs std_srvs)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# 四种激励入口共用本包的状态机和控制话题发布者。
add_executable(identification_fsm src/identification_fsm.cpp
  src/brct1order_excite.cpp src/brct2order_excite.cpp src/brct3order_excite.cpp src/gazebo_physics_excite.cpp)
target_compile_features(identification_fsm PRIVATE cxx_std_14)
target_link_libraries(identification_fsm ${catkin_LIBRARIES})
install(TARGETS identification_fsm RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装 Python 脚本，并由 Catkin 处理可执行入口。
catkin_install_python(PROGRAMS
  scripts/record_motor_speed.py
  scripts/brct2order_process.py
  scripts/brct3order_process.py
  scripts/brct1order_process.py
  scripts/gazebo_physics_process.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/brct1order_plot.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装本行列出的公共头文件或运行资源，保留目录结构。
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# 安装包说明与资源来源说明。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

# ============================ 测试注册 ============================
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_excitation_control test/test_excitation_control.cpp
    src/brct1order_excite.cpp src/brct2order_excite.cpp src/brct3order_excite.cpp src/gazebo_physics_excite.cpp)
  if(TARGET test_excitation_control)
    target_compile_features(test_excitation_control PRIVATE cxx_std_14)
  endif()
  catkin_add_nosetests(test)
endif()
