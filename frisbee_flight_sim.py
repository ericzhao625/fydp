from math import pi
import matplotlib.pyplot as plt

G = -9.81 # [m/s^2] gravity accel
M = 0.175 # [kg] mass of frisbee
RHO = 1.23 # [kg/m^3] density of air
AREA = 0.0568 # [m^2] ??? check this. area of frisbee
CL0 = 0.15 # Lift coefficient at alpha = 0
CLA = 1.4 # Lift coefficient dependent on alpha
CD0 = 0.08 # Drag coeffficient at alpha = 0
CDA = 2.72 # Drag coefficient dependent on alpha
ALPHA0 = -4


def simulate(y0, vx0, vy0, alpha, deltaT):
    cl = CL0 + CLA * alpha * pi / 180
    cd = CD0 + CDA * ((alpha - ALPHA0) * pi / 180) ** 2
    
    x = [0]
    y = [y0]
    vx = [vx0]
    vy = [vy0]
    times = [0]

    while y[-1] > 0:
        delta_vy = (RHO * (vx[-1] ** 2) * AREA * cl / 2 / M + G) * deltaT
        delta_vx = -RHO * (vx[-1] ** 2) * AREA * cd * deltaT
        
        vx.append(vx[-1] + delta_vx)
        vy.append(vy[-1] + delta_vy)
        x.append(x[-1] + vx[-1] * deltaT)
        y.append(y[-1] + vy[-1] * deltaT)

        times.append(deltaT)
    
    return x, y, vx, vy, times
        
if __name__ == "__main__":
    plt.figure()
    x, y, vx, vy, times = simulate(1, 8.5, 0, 12, 0.001)
    plt.ylabel("Height [m]")
    plt.xlabel("Distance [m]")
    plt.scatter(x, y)
    
    plt.figure()
    x, y, vx, vy, times = simulate(1, 14.7, 0, 12, 0.001)
    plt.scatter(x, y)

    plt.ylabel("Height [m]")
    plt.xlabel("Distance [m]")
    plt.show()