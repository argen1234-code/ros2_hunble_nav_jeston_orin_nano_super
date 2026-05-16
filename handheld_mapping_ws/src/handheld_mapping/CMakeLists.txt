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

install(PROGRAMS
  scripts/cmd_vel_sender.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_sender
)

install(PROGRAMS
  scripts/virtual_goal_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME virtual_goal_publisher
)

install(PROGRAMS
  scripts/mqtt_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_bridge
)

install(PROGRAMS
  scripts/stm32_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stm32_bridge
)

install(PROGRAMS
  scripts/find_stm32_port.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
