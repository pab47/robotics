from matplotlib import pyplot as plt
import numpy as np

def forward_kinematics(theta,parms):
    r1, r2, r3 = parms[:3]
    theta1, theta2, theta3 = theta[:3]

    c1, s1 = np.cos(theta1), np.sin(theta1)
    c12, s12 = np.cos(theta1 + theta2), np.sin(theta1 + theta2)
    c123, s123 = np.cos(theta1 + theta2 + theta3), np.sin(theta1 + theta2 + theta3)

    x_R = r1*c1+r2*c12+r3*c123
    y_R = r1*s1+r2*s12+r3*s123
    phi = theta1 + theta2 + theta3

    return x_R, y_R, phi

def visualize(theta,parms):

    r1, r2, r3 = parms[:3]
    theta1, theta2, theta3 = theta[:3]

    c1, s1 = np.cos(theta1), np.sin(theta1)
    c2, s2 = np.cos(theta2), np.sin(theta2)
    c3, s3 = np.cos(theta3), np.sin(theta3)
    
    H1 = np.array([
        [c1, -s1, 0, 0],
        [s1,  c1, 0, 0],
        [0,    0, 1, 0],
        [0,    0, 0, 1]
    ])

    H2 = np.array([
        [c2, -s2, 0, -r1 * (c2 - 1)],
        [s2,  c2, 0, -r1 * s2],
        [0,    0, 1, 0],
        [0,    0, 0, 1]
    ])

    H3 = np.array([
        [c3, -s3, 0, -(r1 + r2) * (c3 - 1)],
        [s3,  c3, 0, -(r1 + r2) * s3],
        [0,    0, 1, 0],
        [0,    0, 0, 1]
    ])

    O = [0, 0]
    P = H1 @ np.array([r1, 0, 0, 1])
    Q = H1 @ H2 @ np.array([r1+r2, 0, 0, 1])
    R = H1 @ H2 @ H3@ np.array([r1+r2+r3, 0, 0, 1])

    # Manipulator looks like this when all angles are zero:
    # O --r1-- P --r2-- Q --r3-- R

    # Draw line from O to P
    plt.plot([O[0], P[0]],[O[1], P[1]],linewidth=5, color='red')

    # Draw line from P to Q
    plt.plot([P[0], Q[0]],[P[1], Q[1]],linewidth=5, color='blue')

    # %Draw line from Q to R 
    plt.plot([Q[0], R[0]],[Q[1], R[1]],linewidth=5, color='green')

    plt.xlabel("x", fontsize=18)
    plt.ylabel("y", fontsize=18)

    plt.xlim(-3,3)
    plt.ylim(-3,3)
    plt.grid()
    plt.gca().set_aspect('equal')
    plt.tick_params(axis='both', labelsize=14)
    
    plt.show()
    

def main():

    #link lengths
    r1, r2, r3 = 1, 1, 0.5
    parms = [r1, r2, r3]
    
    #forward kinematics visualization
    theta = [-np.pi/2,np.pi/2,-np.pi/2]
    visualize(theta,parms)

    #forward kinematics verification
    x_R,y_R,phi = forward_kinematics(theta,parms)
    print(f"End effector position/orientation: x_R,y_R={x_R:.2f},{y_R:.2f}, phi={phi:.2f} radians")
        
if __name__ == "__main__":
    main()
