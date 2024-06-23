define kinect ranger
(
  sensor
  (
    range [0 30]
    fov 360.0
    samples 720
  )
  # generic model properties
  color "black"
  size [0.0 0.0 0.0]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.005 0.005 0.001 0.005 0.003]

  size [0.612 0.58 0.245]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "gray"
  
  kinect()
)
