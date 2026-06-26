cmake_minimum_required(VERSION 3.0.2)
project(indooruav_controller)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  indooruav_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
)

catkin_package(
  INCLUDE_DIRS include psdk_lib/include
  CATKIN_DEPENDS geometry_msgs indooruav_msgs mavros_msgs nav_msgs roscpp std_msgs std_srvs tf
)

# ── Payload SDK 静态库 ─────────────────────────────────────
set(PAYLOAD_SDK_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/lib")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(PAYLOAD_SDK_STATIC_LIB
      "${PAYLOAD_SDK_LIBRARY_DIR}/x86_64-linux-gnu-gcc/libpayloadsdk.a")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(PAYLOAD_SDK_STATIC_LIB
      "${PAYLOAD_SDK_LIBRARY_DIR}/aarch64-linux-gnu-gcc/libpayloadsdk.a")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(armv7l|armv7|arm)$")
  set(PAYLOAD_SDK_STATIC_LIB
      "${PAYLOAD_SDK_LIBRARY_DIR}/arm-linux-gnueabihf-gcc/libpayloadsdk.a")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT EXISTS "${PAYLOAD_SDK_STATIC_LIB}")
  message(FATAL_ERROR "Payload SDK library not found: ${PAYLOAD_SDK_STATIC_LIB}")
endif()

include_directories(
  include
  include/dependences
  psdk_lib/include
  ${catkin_INCLUDE_DIRS}
)

# ── PSDK 平台相关代码 ──────────────────────────────────────
add_library(${PROJECT_NAME}_platform
  src/dependences/application.cpp
  src/dependences/hal_i2c.c
  src/dependences/hal_uart.c
  src/dependences/osal.c
  src/dependences/osal_fs.c
  src/dependences/osal_socket.c
)
add_dependencies(${PROJECT_NAME}_platform
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_platform
  PUBLIC
    ${PAYLOAD_SDK_STATIC_LIB}
    ${catkin_LIBRARIES}
    dl
    gpiod
    pthread
    rt
)

# ── 主节点 ─────────────────────────────────────────────────
add_executable(controller_hardware_node
  node/controller_hardware_node.cpp
  src/indooruav_controller/controller_hardware.cpp
)
add_dependencies(controller_hardware_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(controller_hardware_node
  PRIVATE
    ${PROJECT_NAME}_platform
    ${catkin_LIBRARIES}
)
target_compile_options(controller_hardware_node
  PRIVATE
    -Wall
    -Wextra
)

add_executable(psdk_state_adapter_node
  node/psdk_state_adapter_node.cpp
  src/indooruav_controller/psdk_state_adapter.cpp
)
add_dependencies(psdk_state_adapter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(psdk_state_adapter_node
  PRIVATE
    ${PROJECT_NAME}_platform
    ${catkin_LIBRARIES}
)
target_compile_options(psdk_state_adapter_node
  PRIVATE
    -Wall
    -Wextra
)

add_executable(controller_hardware_with_state_node
  node/controller_hardware_with_state_node.cpp
  src/indooruav_controller/controller_hardware.cpp
  src/indooruav_controller/psdk_state_adapter.cpp
)
add_dependencies(controller_hardware_with_state_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(controller_hardware_with_state_node
  PRIVATE
    ${PROJECT_NAME}_platform
    ${catkin_LIBRARIES}
)
target_compile_options(controller_hardware_with_state_node
  PRIVATE
    -Wall
    -Wextra
)

# ── 仿真节点 ─────────────────────────────────────────────────
add_executable(controller_simulate_node
  node/controller_simulate_node.cpp
  src/indooruav_controller/controller_simulate.cpp
)
add_dependencies(controller_simulate_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(controller_simulate_node
  PRIVATE
    ${catkin_LIBRARIES}
)
target_compile_options(controller_simulate_node
  PRIVATE
    -Wall
    -Wextra
)

# ── 安装 ───────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}_platform
  controller_hardware_node
  controller_simulate_node
  psdk_state_adapter_node
  controller_hardware_with_state_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  scripts/odometry_frame_rotator.py
  scripts/test_controller_hardware.py
  scripts/test_controller_simulate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
