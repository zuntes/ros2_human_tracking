cmake_minimum_required(VERSION 3.8)
project(yolo_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/Mask.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint2DArray.msg"
  "msg/KeyPoint3D.msg"
  "msg/KeyPoint3DArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "msg/Point2D.msg"
  "msg/PointArray.msg"
  "msg/Pose2D.msg"
  "msg/Pose2DArray.msg"
  "msg/Vector2D.msg"
  "msg/Vector3D.msg"

  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
