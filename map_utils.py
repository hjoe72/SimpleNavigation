import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class MapPlotter:
    def __init__(self, vessel, map_size_x, map_size_y, waypoint, init_pos, init_rot_deg):
        self.map_size_x = map_size_x
        self.map_size_y = map_size_y
        
        self.vessel = vessel

        self.fig, self.ax = plt.subplots()
        self.ln = plt.scatter(float(waypoint[0]), float(waypoint[1]), color='b')
        self.n_waypoints = 1
        plt.annotate('1', (float(waypoint[0]+1), float(waypoint[1]+1)))
        self.ani = FuncAnimation(self.fig, self.update_plot, frames=None, init_func=self.initialize_plot, repeat=False, interval=24)
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.map_clicked = False
        self.path_counter = 0
        
        plt.title("Right Click on Map to Add More Waypoints")
        plt.show()
        
    def onclick(self, event):
        if event.button==3:
            self.vessel.add_waypoints(np.array([[event.xdata],[event.ydata]]))
            self.n_waypoints+=1
            self.map_clicked = True
        

    def initialize_plot(self):
        self.ax.set_xlim(-self.map_size_x/2, self.map_size_x/2)
        self.ax.set_ylim(-self.map_size_y/2, self.map_size_y/2)
        x, y, dx, dy = self.vessel.get_plotting_var()
        self.boat_plot = plt.arrow(x, y, dx=2*dx, dy=2*dy, width=0.2, head_width=2, color='r')
        return self.ln
    
    def update_plot(self, frame):
        is_moving = self.vessel.motion_update()
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
            new_waypoint = self.vessel.get_newest_waypoint()
            self.ln = plt.scatter(new_waypoint[0], new_waypoint[1], color='b')
            plt.annotate(str(self.n_waypoints), (new_waypoint[0]+1,new_waypoint[1]+1))
            self.map_clicked = False
        
        return self.ln

