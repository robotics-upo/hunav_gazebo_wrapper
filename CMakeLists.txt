cmake_minimum_required(VERSION 3.8)
project(hunav_gazebo_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include)
include_directories(SYSTEM
  #${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${TINYXML2_LIBRARY_DIRS}
  #${TINYXML2_LIBRARY_PATH}
)


add_library(HuNavPlugin SHARED src/HuNavPlugin.cpp)
target_include_directories(HuNavPlugin PUBLIC include) #${ignition-common3_INCLUDE_DIRS}
ament_target_dependencies(HuNavPlugin 
  rclcpp 
  gazebo 
  gazebo_ros 
  hunav_msgs 
  geometry_msgs
  tf2 
  tf2_geometry_msgs
) 
ament_export_libraries(HuNavPlugin)
target_link_libraries(HuNavPlugin ${GAZEBO_LIBRARIES}) #${ignition-common3_LIBRARIES}


add_executable(hunav_gazebo_world_generator src/WorldGenerator.cpp)
ament_target_dependencies(hunav_gazebo_world_generator rclcpp hunav_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros tinyxml2_vendor)
target_link_libraries(hunav_gazebo_world_generator ${TINYXML2_LIBRARY})

install(TARGETS hunav_gazebo_world_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS HuNavPlugin 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
   DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY media/models
   DESTINATION share/${PROJECT_NAME}/
)
# install(DIRECTORY media/models/textures
#    DESTINATION share/${PROJECT_NAME}/textures
# )


ament_package()
