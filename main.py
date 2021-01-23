import numpy as np
import matplotlib.pyplot as plt
import time
import sys
from scipy.linalg import expm
from matplotlib.animation import FuncAnimation

class Vessel:
    def __init__(self, init_pos, init_rot_deg, waypoint):
        pos_mat = init_pos
        rot_mat = np.array([[np.cos(init_rot_deg), -np.sin(init_rot_deg)],[np.sin(init_rot_deg), np.cos(init_rot_deg)]])
        self.pose = np.append(np.append(rot_mat, pos_mat, axis=1), np.array([[0,0,1]]), axis=0)
        self.ts = time.time()
        self.waypoints = [waypoint]
        print("Current waypoint is ({:.2f}, {:.2f})".format(float(waypoint[0]), float(waypoint[1])))
    
    def get_angular_velocity(self, waypoint_boat_f):
        # Return the angular velocity given goal position, both are in boat frame of reference
        # Angular velocity omega in m/s
        omega = np.arctan(waypoint_boat_f[1]/(waypoint_boat_f[0]+0.00001))
        if (waypoint_boat_f[0]>=0):
            omega*=1
        elif (waypoint_boat_f[0]<0 and waypoint_boat_f[1]<0):
            omega = -np.pi + omega
        else:
            omega = np.pi + omega
        return omega
    	
    def get_translational_velocity(self, waypoint_boat_f):
        # Return the translational velocity given the goal position, both are in boat frame of reference
        # Assume the boat only move straight ahead, velocity is (vel, 0)
        # vel in m/s
        dist = np.linalg.norm(waypoint_boat_f)
        if dist>3:
            vel = 4
        else:
            vel = 0
        return vel
    
    def motion_update(self, waypoint, data=None):
        # Goal position is in world frame
        # Use optional argument data to store any additional data needed such as for planning algorithm
        if waypoint is not None:
    	    waypoint_boat_f = np.matmul(np.linalg.inv(self.pose), np.append(waypoint, np.array([[1]])))
    	
    	    omega = self.get_angular_velocity(waypoint_boat_f)
    	    omega_hat = np.array([[0,-omega],[omega,0]])
    	
    	    dist_to_waypoint = np.linalg.norm(waypoint_boat_f)
    	    if dist_to_waypoint>3:
    	        v = self.get_translational_velocity(waypoint_boat_f)
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
        if len(self.waypoints)==1:
            print("Current waypoint is ({:.2f}, {:.2f})".format(float(waypoint[0]), float(waypoint[1])))
        else:
            print("Added waypoint ({:.2f}, {:.2f})".format(float(waypoint[0]), float(waypoint[1])))
        
    def remove_current_waypoint(self):
        deleted_wp = self.waypoints.pop(0)
        print("Waypoint ({0:.2f}, {0:.2f}) is deleted".format(float(deleted_wp[0]), float(deleted_wp[1])))

class MapPlotter:
    def __init__(self, map_size_x, map_size_y, waypoint, init_pos, init_rot_deg):
        self.map_size_x = map_size_x
        self.map_size_y = map_size_y
        
        self.vessel = Vessel(init_pos, init_rot_deg, waypoint)

        self.fig, self.ax = plt.subplots()
        self.ln = plt.scatter(float(waypoint[0]), float(waypoint[1]), color='b')
        self.n_waypoints = 1
        plt.annotate('1', (float(waypoint[0]+1), float(waypoint[1]+1)))
        self.ani = FuncAnimation(self.fig, self.update_plot, frames=None, init_func=self.initialize_plot, repeat=False, interval=24)
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.map_clicked = False
        self.path_counter = 0
        
    def onclick(self, event):
        self.vessel.add_waypoints(np.array([[event.xdata],[event.ydata]]))
        self.map_clicked = True
        

    def initialize_plot(self):
        self.ax.set_xlim(-map_size_x/2, map_size_x/2)
        self.ax.set_ylim(-map_size_y/2, map_size_y/2)
        x, y, dx, dy = self.vessel.get_plotting_var()
        self.boat_plot = plt.arrow(x, y, dx=2*dx, dy=2*dy, width=0.2, head_width=2, color='r')
        return self.ln
    
    def update_plot(self, frame):
        is_moving = self.vessel.motion_update(self.vessel.get_current_waypoint())
        if is_moving:
            self.boat_plot.remove()
            x, y, dx, dy = self.vessel.get_plotting_var()
            dxdy_norm = np.sqrt(dx**2 + dy**2)/2
        
            self.boat_plot = plt.arrow(x, y, dx=dx/dxdy_norm, dy=dy/dxdy_norm, width=0.2, head_width=2, color='r')
            self.path_counter+=1
            self.path_counter%=10
            if self.path_counter==0:
                plt.scatter(x, y, marker='.', color='r')
        
        if self.map_clicked:
            self.n_waypoints+=1
            new_waypoint = self.vessel.get_newest_waypoint()
            self.ln = plt.scatter(new_waypoint[0], new_waypoint[1], color='b')
            plt.annotate(str(self.n_waypoints), (new_waypoint[0]+1,new_waypoint[1]+1))
            self.map_clicked = False
        
        return self.ln
    
        
    

# Map size in meters
map_size_x = 200
map_size_y = 100

# Initial position and rotation of the robot in world frame
# Rotation is measured by radian clockwise from East
# Try to use different values
init_rot_deg = np.pi/6
init_pos = np.array([[0],[0]])

# Waypoint for the robot in world frame
waypoint = np.array([[-10],[20]])

map_plotter = MapPlotter(map_size_x, map_size_y, waypoint, init_pos, init_rot_deg)

plt.title("Click on Map to Add More Waypoints")
plt.show()
