from matplotlib import pyplot as plt
import numpy as np
from scipy import interpolate
from scipy.integrate import odeint
import mpl_toolkits.mplot3d.axes3d as p3

class parameters:
    def __init__(self):
        self.m = 0.468
        self.I = np.array([
            [4.856e-3, 1e-5,    1e-5],
            [1e-5,    4.856e-3, 1e-5],
            [1e-5,    1e-5,    8.801e-3]
            ])
        self.g = 9.81
        self.l = 0.225
        self.kf = 2.980*1e-6
        self.km = 1.14*1e-7
        self.pause = 0.01
        self.fps = 60

def R(phi,theta,psi):

    R_x = np.array([[1,            0,         0],
                    [0,     np.cos(phi), -np.sin(phi)],
                    [0,     np.sin(phi),  np.cos(phi)]])

    R_y = np.array([[np.cos(theta),  0, np.sin(theta)],
                        [0,           1,          0],
                        [-np.sin(theta),  0, np.cos(theta)]])

    R_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                   [np.sin(psi),  np.cos(psi), 0],
                   [0,            0,         1]])

    #rotation matrix in z-y-x convention
    return R_z @ R_y @ R_x

def animate(t, Xpos, Xang,parms):

    # Get parameters
    l = parms.l
    
    # Interpolate position and angles for smoother animation
    Xpos = np.array(Xpos)
    Xang = np.array(Xang)
    t_interp = np.arange(t[0], t[-1], 1 / parms.fps)
    Xpos_interp = np.zeros((len(t_interp), Xpos.shape[1]))
    Xang_interp = np.zeros((len(t_interp), Xang.shape[1]))
    for i in range(Xpos.shape[1]):
        fpos = interpolate.interp1d(t, Xpos[:, i])
        Xpos_interp[:, i] = fpos(t_interp)
        fang = interpolate.interp1d(t, Xang[:, i])
        Xang_interp[:, i] = fang(t_interp)

    # Setting up the figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([-0.5, 0.5])

    # Define the initial state of the axles
    axle1, = ax.plot([], [], [], 'ro-', linewidth=3)
    axle2, = ax.plot([], [], [], 'bo-', linewidth=3)

    # Animation loop
    for ii in range(len(t_interp)):
        # Clear the axis for the next frame
        ax.cla()

        # Set the axes limits and view again after clearing
        ax.set_xlim([-0.5, 0.5])
        ax.set_ylim([-0.5, 0.5])
        ax.set_zlim([-0.5, 0.5])
        ax.view_init(azim=-72, elev=20)

        # Get the interpolated position and orientation
        x, y, z = Xpos_interp[ii]
        phi, theta, psi = Xang_interp[ii]
    
        # Define the axles in the body frame and transform to the world frame
        axle_x = np.array([[-l, 0, 0], [l, 0, 0]])
        axle_y = np.array([[0, -l, 0], [0, l, 0]])

        new_axle_x = (R(phi, theta, psi) @ axle_x.T).T + np.array([x, y, z])
        new_axle_y = (R(phi, theta, psi) @ axle_y.T).T + np.array([x, y, z])

        # Update axle data
        axle1.set_data(new_axle_x[:, 0], new_axle_x[:, 1])
        axle1.set_3d_properties(new_axle_x[:, 2])
        axle2.set_data(new_axle_y[:, 0], new_axle_y[:, 1])
        axle2.set_3d_properties(new_axle_y[:, 2])

        # Replot the axles
        ax.plot(new_axle_x[:, 0], new_axle_x[:, 1], new_axle_x[:, 2], 'ro-', linewidth=3)
        ax.plot(new_axle_y[:, 0], new_axle_y[:, 1], new_axle_y[:, 2], 'bo-', linewidth=3)

        # Pause to show animation
        plt.pause(parms.pause)

    plt.close()

def control(X,t, parms):
    m, g, kf = parms.m, parms.g, parms.kf
    u0 = m * g / (4 * kf)
    f = 0.1

    mode = "none"  # options: "none", "hover", "thrust", "roll", 
                   #            "pitch", "yaw","lateral" 
    
    if mode == "none":
        omega1_sq = omega2_sq = omega3_sq = omega4_sq = 0

    elif mode == "hover":
        omega1_sq = omega2_sq = omega3_sq = omega4_sq = u0

    elif mode == "thrust":
        omega1_sq = omega2_sq = omega3_sq = omega4_sq = (1 + f) * u0

    elif mode == "roll":
        omega1_sq = u0
        omega2_sq = (1 - f) * u0
        omega3_sq = u0
        omega4_sq = (1 + f) * u0

    elif mode == "pitch":
        omega1_sq = (1 - f) * u0
        omega2_sq = u0
        omega3_sq = (1 + f) * u0
        omega4_sq = u0

    elif mode == "yaw":
        omega1_sq = (1 - f) * u0
        omega2_sq = (1 + f) * u0
        omega3_sq = (1 - f) * u0
        omega4_sq = (1 + f) * u0
        
    elif mode == "lateral":
        if (t<0.1): #1) Pitch forward
            omega1_sq = (1-f)*u0
            omega2_sq = u0
            omega3_sq = (1+f)*u0
            omega4_sq = u0    
        elif(t>0.1 and t<0.2): #2) Thrust control
            
            omega1_sq = (1+f)*u0
            omega2_sq = (1+f)*u0
            omega3_sq = (1+f)*u0
            omega4_sq = (1+f)*u0
        elif(t>0.2 and t<0.33): #3) Pitch back  
            omega1_sq = (1+f)*u0
            omega2_sq = u0
            omega3_sq = (1-f)*u0
            omega4_sq = u0
        else: #4) hovering control 
            omega1_sq = u0
            omega2_sq = u0
            omega3_sq = u0
            omega4_sq = u0
    
    else:
        omega1_sq = omega2_sq = omega3_sq = omega4_sq = 0


    return omega1_sq, omega2_sq, omega3_sq, omega4_sq
 
def quadrotor_equations(X,t,parms):

    # get parameters
    m,I,g,l = parms.m,parms.I,parms.g,parms.l
    kf,km = parms.kf,parms.km

    #get state variables
    #x= X[0]; y = X[1]; z = X[2] #not needed in this function; written for clarity
    phi = X[3]; theta = X[4]; psi = X[5]
    v_x = X[6]; v_y = X[7]; v_z = X[8]
    omega_x = X[9]; omega_y = X[10]; omega_z = X[11]   

    #get control inputs
    omega1_sq,omega2_sq,omega3_sq,omega4_sq = control(X,t,parms)

    #compute the external forces and torques in the body frame
    F_z = kf*(omega1_sq+omega2_sq+omega3_sq+omega4_sq)
    tau_phi = kf*l*(omega4_sq - omega2_sq)
    tau_theta = kf*l*(omega3_sq - omega1_sq)
    tau_psi = km*(omega1_sq-omega2_sq+omega3_sq-omega4_sq)
    F_ext = R(phi,theta,psi) @  np.array([0, 0, F_z])
    tau_ext = np.array([tau_phi,tau_theta,tau_psi])  

    #compute the linear and angular accelerations
    v_dot = F_ext/m - np.array([0, 0, g]) 
    vx_dot = v_dot[0]
    vy_dot = v_dot[1]
    vz_dot = v_dot[2]

    omega = np.array([omega_x, omega_y, omega_z]) #set angular velocity of the drone in body frame
    omega_dot = np.linalg.inv(I) @ (tau_ext - np.cross(omega, I @ omega))
    omega_x_dot = omega_dot[0]
    omega_y_dot = omega_dot[1]
    omega_z_dot = omega_dot[2]

    #compute the Euler angle rates
    T = np.array([
        [1.0,  np.sin(phi) * np.tan(theta),  np.cos(phi) * np.tan(theta)],
        [0.0,  np.cos(phi), -np.sin(phi)],
        [0.0,  np.sin(phi) / np.cos(theta),  np.cos(phi) / np.cos(theta)]
    ])
    euler_rate = T @ omega
    phidot = euler_rate[0]
    thetadot = euler_rate[1]
    psidot = euler_rate[2]

    #return the state derivatives for integration
    return np.array([v_x, v_y, v_z, phidot, thetadot, psidot, \
                    vx_dot, vy_dot, vz_dot, omega_x_dot, omega_y_dot, omega_z_dot])

def main():
    #initialize parameters
    parms = parameters()

    #initial conditions
    x0,y0,z0 = 0, 0, 0
    phi0,theta0,psi0 = 0, 0, 0
    v_x0,v_y0,v_z0 = 0, 0, 0
    omega_x0,omega_y0,omega_z0 = 0, 0, 0
    X0 = np.array([x0, y0, z0, phi0, theta0, psi0, v_x0, v_y0, v_z0, omega_x0, omega_y0, omega_z0])
    
    # Simulate the dynamics
    t = np.linspace(0, 1, 101)
    X = odeint(quadrotor_equations, X0, t, args=(parms,))

    #postprocess the results for animation
    X_pos,X_ang = X[:, 0:3], X[:, 3:6]
    animate(t,X_pos,X_ang,parms)

if __name__ == "__main__":
    main()

