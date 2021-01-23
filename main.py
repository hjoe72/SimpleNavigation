import numpy as np
import argparse
import vessel
import map_utils
import vessel

parser = argparse.ArgumentParser()
parser.add_argument('--map_size_x', type=float, default=200, help='Map x size in meters')
parser.add_argument('--map_size_y', type=float, default=100, help='Map y size in meters')
parser.add_argument('--init_rot', type=float, default=np.pi/6, help='Initial vessel orientation in degree')
parser.add_argument('--init_pos_x', type=float, default=0, help='Initial vessel x position')
parser.add_argument('--init_pos_y', type=float, default=0, help='Initial vessel y position')
parser.add_argument('--init_wp_x', type=float, default=-10, help='First waypoint x position')
parser.add_argument('--init_wp_y', type=float, default=20, help='First waypoint y position')


args = parser.parse_args()

# Map size in meters
map_size_x = args.map_size_x
map_size_y = args.map_size_y


# Initial position and rotation of the robot in world frame
# Rotation is measured by radian clockwise from East
# Try to use different values
init_rot_deg = args.init_rot
init_pos = np.array([[args.init_pos_x],[args.init_pos_y]])

# Initial waypoint for the robot in world frame
init_waypoint = np.array([[args.init_wp_x],[args.init_wp_y]])

#Initialize the vessel
vessel = vessel.Vessel(init_pos, init_rot_deg, init_waypoint)

map_plotter = map_utils.MapPlotter(vessel, map_size_x, map_size_y, init_waypoint, init_pos, init_rot_deg)
