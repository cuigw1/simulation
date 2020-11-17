define kinect ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)
define laser ranger
(
  sensor
  (
    range_max 30
    fov 230
    samples 800
  )
  color "black"
  size [0.06 0.15 0.03]
)


define diff_robot  position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  size [0.55 0.5 0.25 ]  
  
  localization "odom"
  odom_error [0 0 0 0 0 0]
  
  gui_nose 1
  drive "diff"
  color "black"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  laser(pose [0.2 0.000 0.000 0.000])
)
