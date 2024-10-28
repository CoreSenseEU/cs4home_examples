cmake_minimum_required(VERSION 3.8)
project(cs4home_simple_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-fsanitize=address)
# add_link_options(-fsanitize=address)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cs4home_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hri_msgs REQUIRED)
find_package(knowledge_graph REQUIRED)
find_package(backward_ros REQUIRED)

set (dependencies 
  cs4home_core
  rclcpp
  rclcpp_components
  hri_msgs
  knowledge_graph
  backward_ros
)

include_directories(include)

add_library(face_afferent SHARED src/cs4home_simple_project/FaceAfferent.cpp)
ament_target_dependencies(face_afferent ${dependencies})

add_library(face_efferent SHARED src/cs4home_simple_project/FaceEfferent.cpp)
ament_target_dependencies(face_efferent ${dependencies})

add_library(face_core SHARED src/cs4home_simple_project/FaceCore.cpp)
ament_target_dependencies(face_core ${dependencies})

add_executable(face_module src/FaceModule.cpp)
ament_target_dependencies(face_module ${dependencies})
target_link_libraries(face_module 
  face_afferent
  face_efferent
  face_core
)

install(TARGETS face_afferent face_efferent face_core face_module
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
