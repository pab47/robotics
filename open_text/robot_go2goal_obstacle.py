from matplotlib import pyplot as plt
from scipy import interpolate
import random
import numpy as np

class parameters:
    def __init__(self):

        #simulation, animation, robot parameters
        self.dt = 0.1; #integration step
        self.N = 500 #maximum time steps to reach the goal, max_time = N*dt
        self.R = 0.1; #radius of the robot
        self.arena_size = 3 #size of the scene
        self.pause = 0.0001 #parameters for animation
        self.fps = 5 #parameters for animation
        
        #obstacle parameters
        self.r_obs = 0.2; #radius of obstacles (for auto mode)
        self.obstacles = np.empty((0, 3)) #initialize empty array for obstacles
        self.obs_mode ='auto' #options: 'none', 'manual', 'auto' (none: no obstacles, manual: user defined obstacles, auto: randomly generated obstacles)
        
        #controller parameters
        self.goal = np.array([2,2]) #goal position
        self.rgoal = 0.1 #when the robot is at this distance from goal then stop
        self.control_mode = 'blend' #options: 'blend', 'switch'
        self.v0 = 0.5; #nominal speed
        self.K = 10; #gain for turning
        self.r0 = 0.2; #when the robot is at this distance from goal then stop
        self.r_buffer = self.R; #buffer for avoiding obstacle
        self.obs_influence_dist = 1.0 # distance at which obstacles start influencing the robot

def animate(t,z,parms):

    t_interp = np.arange(t[0],t[len(t)-1],1/parms.fps)
    [m,n] = np.shape(z)
    shape = (len(t_interp),n)
    z_interp = np.zeros(shape)

    for i in range(0,n):
        f = interpolate.interp1d(t, z[:,i])
        z_interp[:,i] = f(t_interp)

    R = parms.R
    phi = np.arange(0,2*np.pi,0.1)

    x_goal = parms.goal[0]
    y_goal = parms.goal[1]

    n_obstacles,_ = parms.obstacles.shape

    for i in range(0,len(t_interp)):
        x = z_interp[i,0]
        y = z_interp[i,1]
        theta = z_interp[i,2]

        x_robot = x + R*np.cos(phi)
        y_robot = y + R*np.sin(phi)

        x2 = x + R*np.cos(theta)
        y2 = y + R*np.sin(theta)

        line, = plt.plot([x, x2],[y, y2],color="black")
        robot,  = plt.plot(x_robot,y_robot,color='black')
        shape, = plt.plot(z_interp[0:i,0],z_interp[0:i,1],color='blue');
        plt.plot(x_goal, y_goal, 'ko', markersize=10, markerfacecolor='black')

        if n_obstacles > 0:
            for i in range(0,n_obstacles):
                x_center_obs = parms.obstacles[i,0];
                y_center_obs = parms.obstacles[i,1];
                r_obs = parms.obstacles[i,2];

                x_obs = x_center_obs + r_obs*np.cos(phi)
                y_obs = y_center_obs + r_obs*np.sin(phi)

                plt.plot(x_obs,y_obs,color='red')

        plt.xlim(-parms.arena_size,parms.arena_size)
        plt.ylim(-parms.arena_size,parms.arena_size)
        plt.gca().set_aspect('equal')
        plt.pause(parms.pause)
        line.remove()
        robot.remove()
        shape.remove()

    plt.show()

def euler_integration(tspan,z0,u,parms):
    v = u[0]
    omega = u[1]
    dt = tspan[1]-tspan[0]

    x_current = z0[0]
    y_current = z0[1]
    theta_current = z0[2]

    xdot = v*np.cos(theta_current)
    ydot = v*np.sin(theta_current)
    thetadot = omega

    x_new = x_current + xdot*dt
    y_new = y_current + ydot*dt
    theta_new = theta_current + thetadot*dt

    return [x_new, y_new, theta_new]

def create_obstacles(type='auto',arena_size=3,R=0.1,r_obs=0.2,n_obstacles=20):
    
    if (type == "none"): #no obstacles
        obstacles = np.empty((0, 3))
    elif (type == 'manual'): #manually defined obstacles (x, y, radius)
        obstacles = np.array([
        [1,1,0.2],
        [1,0.5,0.2],
        [0,1,0.2],
        [-1,0.5,0.2]
        ])
    else: #randomly generated obstacles
        obstacles = np.empty((0, 3))
        r_min = R+0.5
        r_max = arena_size-0.5
        for i in range(0,n_obstacles):
            r = random.uniform(r_min,r_max)
            theta_obs = random.uniform(-np.pi,np.pi)
            x_obs = r*np.cos(theta_obs)
            y_obs = r*np.sin(theta_obs)
            tmp = np.array([x_obs,y_obs,r_obs])
            obstacles = np.vstack([obstacles, tmp])

    return obstacles 

def control(z,parms):
        
        # --- Current state ---
        x = z[0]; y = z[1]; theta = z[2]

        # --- Goal heading ---
        theta_goal = np.arctan2(parms.goal[1]-y, parms.goal[0]-x)

        # --- Default: no obstacle influence ---
        theta_obs = theta_goal
        w_obs = 0.0

        n_obstacles,_ = parms.obstacles.shape
        if n_obstacles > 0:
        
            min_dist = np.inf
            closest_obs = None

            # --- Find closest obstacle (signed distance) ---
            for i in range(n_obstacles):
                x_obs = parms.obstacles[i, 0]
                y_obs = parms.obstacles[i, 1]
                r_obs = parms.obstacles[i, 2] 

                dist = np.sqrt((x_obs - x)**2 + (y_obs - y)**2) - r_obs - parms.r_buffer

                if dist < min_dist:
                    min_dist = dist
                    closest_obs = (x_obs, y_obs)
                    

            # --- Compute obstacle heading if relevant ---
            if closest_obs is not None:
                x_obs, y_obs = closest_obs

                # same structure as your original (perpendicular avoidance)
                theta_obs = np.arctan2(y_obs - y, x_obs - x) -  np.pi / 2

                # --- Smooth proximity-based weight ---
                # sigma controls how early avoidance activates
                sigma = 0.9*parms.r_buffer #parms.obs_influence_dist   # you define this (e.g., 1.0–2.0 m)

                w_obs = np.exp(-(min_dist**2) / (sigma**2))

                # clamp for safety (optional)
                w_obs = np.clip(w_obs, 0.0, 1.0)

        # --- Complementary weight ---
        w_goal = 1.0 - w_obs

        if (parms.control_mode == 'blend'):
            # --- Vector blending ---
            sin_eh = w_goal * np.sin(theta_goal) + w_obs * np.sin(theta_obs)
            cos_eh = w_goal * np.cos(theta_goal) + w_obs * np.cos(theta_obs)
            theta_des = np.arctan2(sin_eh, cos_eh)
        elif (parms.control_mode == 'switch'):
            # -- Hard switching --
            if (min_dist<0):
                theta_des = theta_obs
            else:
                theta_des = theta_goal
        else:
            raise ValueError("Invalid control mode. Choose 'blend' or 'switch'.")

        # --- Control ---
        omega = parms.K * (theta_des - theta)
        v = parms.v0

        # --- Check if goal is reached ---
        r = np.sqrt(  (parms.goal[0] - x)**2 + (parms.goal[1] - y)**2)
        if (r < parms.rgoal):
            v = [] #exit condition: if robot is within r0 distance from goal then stop
            

        return v, omega
   
def main():
    random.seed(2) #seed for reproducibility
    #1) generate the scene and obstacles
    parms = parameters()

    #a) set control_mode:  'blend' for blended control, 
    # 'switch' for hard switching between goal and obstacle avoidance.
    parms.control_mode = 'blend' 

    #b) set obstacle mode: 'none' for no obstacles, 
    # 'manual' for user defined obstacles, 
    # 'auto' for randomly generated obstacles
    parms.obs_mode = 'none'
    no_of_obstacles = 15 #this is ignored in none/manual mode
    parms.obstacles = create_obstacles(parms.obs_mode,parms.arena_size,parms.R,parms.r_obs,n_obstacles=no_of_obstacles) #create obstacles based on type
     
    #2) simulate
    z0 = [-2, -2, 0] #initial condition, [x0, y0, theta0]
    z = np.array([z0])
    t = np.array([0])
    for i in range(0,parms.N):

        # --- Compute control ---
        v,omega = control(z0,parms)

        if (v == []):
            print("Goal reached!")
            break

        # --- Integrate ---
        z0 = euler_integration([0, parms.dt],z0,[v,omega],parms)
        z = np.vstack([z, z0])
        t = np.append(t,t[-1]+parms.dt)

    #3) animate the trajectory
    animate(t,z,parms)

if __name__ == "__main__":
    main()
