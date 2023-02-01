cmake_minimum_required(VERSION 3.0.2)
project(librarian_resources)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
  apriltag_ros
)

add_message_files(
  FILES
  BookShelfInfo.msg
  BookPolys.msg
  BookDetections.msg
  ShelfImage.msg
  ShelfImages.msg
  TextRecognition.msg
  TextRecognitions.msg
  BookRecognition.msg
  BookRecognitions.msg
  Book.msg
  Books.msg
  Environment.msg
  Manipulate.msg
  BookPlacement.msg
  VisionControl.msg
)
add_service_files(
    FILES
    EnvironmentService.srv
    ManipulationService.srv
    ToggleVisionNode.srv
    BookCoverInspect.srv
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  apriltag_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES librarian_resources
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS std_msgs
)

## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

