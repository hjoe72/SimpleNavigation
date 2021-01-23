import numpy as np

class SimpleNavigation:
	def __init__(self):
		pass
	
	def get_angular_velocity(self, waypoint_boat_f):
		# waypoint_boat_f is matrix (N,2) where N is number of waypoints
		# Return the angular velocity given goal position, both are in boat frame of reference
		# Angular velocity omega in m/s
		next_waypoint_boat_f = waypoint_boat_f[:,0]
		omega = np.arctan(next_waypoint_boat_f[1]/(next_waypoint_boat_f[0]+0.00001))
		if (next_waypoint_boat_f[0]>=0):
			omega*=1
		elif (next_waypoint_boat_f[0]<0 and next_waypoint_boat_f[1]<0):
			omega = -np.pi + omega
		else:
			omega = np.pi + omega
		return omega
	
	def get_translational_velocity(self, waypoint_boat_f):
		# waypoint_boat_f is matrix (N,2) where N is number of waypoints
		# Return the translational velocity given the goal position, both are in boat frame of reference
		# Assume the boat only move straight ahead, velocity is (vel, 0)
		# vel in m/s
		next_waypoint_boat_f = waypoint_boat_f[:,0]
		dist = np.linalg.norm(next_waypoint_boat_f)
		if dist>3:
			vel = 4
		else:
			vel = 0
		return vel
