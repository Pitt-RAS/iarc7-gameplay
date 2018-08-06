import rospy
from iarc7_msgs.msg import Boundary
from nav_msgs.msg import Odometry

class ArenaPositionEstimator(object):
    def __init__(self):
        # The position in the arena that the drone will start from
        # e.g. the offset from map 0,0 to arena 0,0 (center of arena)
        arena_starting_position_x = rospy.get_param('~arena_starting_position_x')
        arena_starting_position_y = rospy.get_param('~arena_starting_position_y')

        self._size_x = rospy.get_param('~arena_size_x')
        self._size_y = rospy.get_param('~arena_size_y')

        # left, right, top, bottom
        self.arena_line_positions = [-arena_starting_position_y + self._size_y/2,
                                     -(arena_starting_position_y + self._size_y/2),
                                     -arena_starting_position_x + self._size_x/2,
                                     -(arena_starting_position_x + self._size_x/2)]

        print 'STARTING LINE POSITIONS', self.arena_line_positions
        self.odom = None

        rospy.Subscriber('/floor_detector/boundaries', Boundary, self.boundary_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

    def odom_callback(self, data):
        self.odom = data

    def boundary_callback(self, data):
        return
        if data.boundary_type == 0:
            self.arena_line_positions[0] = data.position
            self.arena_line_positions[1] = data.position - 20.0
        elif data.boundary_type == 1:
            self.arena_line_positions[1] = data.position
            self.arena_line_positions[0] = data.position + 20.0
        elif data.boundary_type == 2:
            self.arena_line_positions[2] = data.position
            self.arena_line_positions[3] = data.position - 20.0
        elif data.boundary_type == 3:
            self.arena_line_positions[3] = data.position
            self.arena_line_positions[2] = data.position + 20.0

    def arena_to_map(self, arena_pos):
        # Arena pos (0,0) is the center of the arena
        from_left_corner = (arena_pos[0] + self._size_x/2, - arena_pos[1] + self._size_y/2)
        return (self.arena_line_positions[3] + from_left_corner[0], self.arena_line_positions[0] - from_left_corner[1])

    def map_to_arena(self, map_pos):
        # Get the coordinates of our object with left corner as origin
        from_left_corner = (map_pos[0] - self.arena_line_positions[3], map_pos[1] - self.arena_line_positions[0])
        # shift our origin to the arena frame by subtracting 10x and adding 10y
        return (from_left_corner[0] - 10, from_left_corner[1] + 10)

    def distance_to_left(self):
        if self.odom is not None:
            dist = self.arena_line_positions[0] - self.odom.pose.pose.position.y
            return dist

    def distance_to_right(self):
        if self.odom is not None:
            dist = self.odom.pose.pose.position.y - self.arena_line_positions[1]
            return dist

    def distance_to_top(self):
        if self.odom is not None:
            dist = self.arena_line_positions[2] - self.odom.pose.pose.position.x
            return dist

    def distance_to_bottom(self):
        if self.odom is not None:
            dist = self.odom.pose.pose.position.x - self.arena_line_positions[3]
            return dist
