cmake_minimum_required(VERSION 3.0.2)
project(jackal_socialsim)

find_package(catkin REQUIRED COMPONENTS

)

catkin_package(
  
)

catkin_python_setup()


catkin_install_python(PROGRAMS
  scripts/condition_check.py
  scripts/data_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)