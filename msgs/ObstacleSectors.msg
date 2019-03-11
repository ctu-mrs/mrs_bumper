std_msgs/Header header
uint32    n_horizontal_sectors  # number of horizontal sectors (total number of sectors -2)
float64   sectors_vertical_fov  # vertical FOV of the horizontal sectors
float64[] sectors               # distance to closest detected obstacle in each sector
int8[]    sector_sensors        # the sensor, used for obstacle detection in each sector
# The horizontal sectors are indexed counterclockwise, starting from the front facing
# sector, relative to the MAV orientation (see picture below). The last two
# elements of the *sectors* variable are the above and below sectors, respectively.
#
#         TOP VIEW
#         --------
#
#        \         /
#         \   0   /   (forward)
#          \     /        x 
#                         ^
#            O O          |
#        1    X    3   <--* z
#            O O       y
#                
#          /     \
#         /   2   \
#        /         \

## Obstacle distance special values
float64 OBSTACLE_UNKNOWN=-1

## Sensor types enum
int8 SENSOR_NONE=-1
int8 SENSOR_DEPTH=0
int8 SENSOR_LIDAR_1D=1
int8 SENSOR_LIDAR_2D=2
