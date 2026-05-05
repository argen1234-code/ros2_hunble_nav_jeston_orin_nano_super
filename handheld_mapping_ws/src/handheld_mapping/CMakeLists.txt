cmake_minimum_required(VERSION 3.5)
project(handheld_mapping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR scripts)

install(DIRECTORY
  launch
  params
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_saver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_saver
)

install(PROGRAMS
  scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_map
)

install(PROGRAMS
  scripts/cmd_vel_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_monitor
)

install(PROGRAMS
  scripts/cmd_vel_watch.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_watch
)

ament_package()
