define test ranger
(
  sensor
  (
    range_max 6.5
    fov 120.0
    samples 640
  )


  color "black"
  size [ 0.06 0.15 0.03 ]
)

define obstacle_dynamic position
(
	pose [ 0.0 0.0 0.0 0.0 ]
	size [0.1 0.1 1.0]
	gui_noise 1.0
	
	drive "omni"
	color "red"
)


define obstacle_static model
(
	size [0.4 0.4 1.0]
	gui_noise 1.0
	color "blue"
)

