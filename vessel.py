import numpy as np
import time
from scipy.linalg import expm
import nav_algorithm

class Vessel:
	def __init__(self, init_pos, init_rot_deg, waypoint):
		pos_mat = init_pos
		rot_mat = np.array([[np.cos(init_rot_deg), -np.sin(init_rot_deg)],[np.sin(init_rot_deg), np.cos(init_rot_deg)]])
		self.pose = np.append(np.append(rot_mat, pos_mat, axis=1), np.array([[0,0,1]]), axis=0)
		self.ts = time.time()
		self.waypoints = [waypoint]
		self.waypoints_matrix = np.asarray(self.waypoints).squeeze(2).transpose() #(N,2)
		self.nav = nav_algorithm.SimpleNavigation()
		print("Current waypoint is ({:.2f}, {:.2f})".format(float(waypoint[0]), float(waypoint[1])))
	
	def motion_update(self, map_data=None):
		# Goal position is in world frame
		# Use optional argument data to store any additional data needed such as for planning algorithm
		if len(self.waypoints)>0:
			waypoint_boat_f = np.matmul(np.linalg.inv(self.pose), np.append(self.waypoints_matrix, np.ones((1, self.waypoints_matrix.shape[1])), axis=0))
			
			omega = self.nav.get_angular_velocity(waypoint_boat_f)
			omega_hat = np.array([[0,-omega],[omega,0]])
			
			dist_to_waypoint = np.linalg.norm(waypoint_boat_f[:,0])
			if dist_to_waypoint>3:
				v = self.nav.get_translational_velocity(waypoint_boat_f)
				pose_vel = np.append(np.append(omega_hat, np.array([[v],[0]]), axis=1), np.array([[0,0,0]]), axis=0)
				
				dt = time.time() - self.ts
				self.ts = time.time()
				self.pose = np.matmul(self.pose, expm(dt*pose_vel))
			
			else:
				self.arrived_at_waypoint()
			
			return True
		else:
			return False
	
	
	def get_plotting_var(self):
		return float(self.pose[0,2]), float(self.pose[1,2]), float(self.pose[0,0]), float(self.pose[1,0])
	
	def get_newest_waypoint(self):
		return self.waypoints[-1]
	
	def get_current_waypoint(self):
		if len(self.waypoints)>0:
			return self.waypoints[0]
		else:
			return None
	
	def arrived_at_waypoint(self):
		print("Arrived at current waypoint")
		self.remove_current_waypoint()
		if len(self.waypoints)>0:
			print("Current waypoint is ({:.2f}, {:.2f})".format(float(self.waypoints[0][0]), float(self.waypoints[0][1])))
		else:
			print("There is no more waypoints")
	
	def add_waypoints(self, waypoint):
		self.waypoints.append(waypoint)
		self.waypoints_matrix = np.asarray(self.waypoints).squeeze(2).transpose()
		if len(self.waypoints)==1:
			print("Current waypoint is ({:.2f}, {:.2f})".format(float(waypoint[0]), float(waypoint[1])))
		else:
			print("Added waypoint ({:.2f}, {:.2f})".format(float(waypoint[0]), float(waypoint[1])))
	
	def remove_current_waypoint(self):
		deleted_wp = self.waypoints.pop(0)
		if len(self.waypoints)>0:
			self.waypoints_matrix = np.asarray(self.waypoints).squeeze(2).transpose()
		print("Waypoint ({0:.2f}, {0:.2f}) is deleted".format(float(deleted_wp[0]), float(deleted_wp[1])))

