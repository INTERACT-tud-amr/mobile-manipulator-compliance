cmake_minimum_required(VERSION 3.0.2)
project(control_interface)

find_package(catkin REQUIRED user_interface_msg dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/CartesianCompliance.cfg
)

catkin_package()

catkin_install_python(PROGRAMS control_interface/control_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
