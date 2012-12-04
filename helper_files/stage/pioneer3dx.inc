# Desc: Device definitions for Activemedia robots.
# Author: Richard Vaughan, Andrew Howard,  Luis Riazuelo
# Date: 10 Jun 2002
# CVS: $Id: pioneer.inc,v 1.30 2008-01-15 01:25:42 rtv Exp $


#  prototype for p3dx sonar sensors
define p3dxsonar sensor
(
  # define the size of each transducer [xsize ysize] in meters
  size [0.01 0.04 0.01]
  # define the field of view of each transducer [range_min range_max view_angle]
  view [0.1 5.0 30]  # min (m), max (m), field of view (deg)
)


# The Pioneer3DX sonar array
define p3dx_sonar ranger
(
  p3dxsonar ( pose [ 0.069 0.136 0 90 ] )
  p3dxsonar ( pose [ 0.114 0.119 0 50 ] )
  p3dxsonar ( pose [ 0.148 0.078 0 30 ] )
  p3dxsonar ( pose [ 0.166 0.027 0 10 ] )
  p3dxsonar ( pose [ 0.166 -0.027 0 -10 ] )
  p3dxsonar ( pose [ 0.148 -0.078 0 -30 ] )
  p3dxsonar ( pose [ 0.114 -0.119 0 -50 ] )
  p3dxsonar ( pose [ 0.069 -0.136 0 -90 ] )
  p3dxsonar ( pose [ -0.157 -0.136 0 -90 ] )
  p3dxsonar ( pose [ -0.203 -0.119 0 -130 ] )
  p3dxsonar ( pose [ -0.237 -0.078 0 -150 ] )
  p3dxsonar ( pose [ -0.255 -0.027 0 -170 ] )
  p3dxsonar ( pose [ -0.255 0.027 0 170 ] )
  p3dxsonar ( pose [ -0.237 0.078 0 150 ] )
  p3dxsonar ( pose [ -0.103 0.119 0 130 ] )
  p3dxsonar ( pose [ -0.157 0.136 0 90 ] )
)


define pioneer_base position 
(
  color "red"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  ranger_return 0.5            	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  #localization "gps"         	  
  #localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  odom_error [ 0.05 0.05 0.0 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)   

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]				
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]					
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)



# The Pioneer3DX standard configuration
define pioneer3dx pioneer_base
(
  # Actual size
  size [0.511 0.4 0.22 ]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0 0.0]

  # Estimated mass in KG
  mass 23.0 

  # Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )

  # Use the sonar array defined above
  #p3dx_sonar( pose [ 0 0 -0.03 0 ] )  
) 
