std_msgs/Header header
uint32    n_horizontal_sectors  # number of horizontal sectors (total number of sectors -2)
float64   sectors_vertical_fov  # vertical FOV of the horizontal sectors
float64[] sectors               # distance to closest detected obstacle in each sector
# The horizontal sectors are indexed clockwise, starting from the front facing
# sector, relative to the MAV orientation (see picture below). The last two
# elements of the *sectors* variable are the above and below sectors, respectively.
#
#         TOP VIEW
#         --------
#
#        \         /
#         \   0   /
#          \     /     forward
#                         ^
#            O O          |
#        3    X    1
#            O O
#                
#          /     \
#         /   2   \
#        /         \
