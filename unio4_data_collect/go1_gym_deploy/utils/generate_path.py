import numpy as np
import matplotlib.pyplot as plt

class VelocityProfile:
    def __init__(self, T, t, dt=0.02):
        self.T = T
        self.dt = dt 
        self.time = t

    def generate_trapezoid(self, v_x=0.5, v_y=0.0, v_omega=0.0, acc_ratio=0.5):
        T_acc = self.T * acc_ratio
        T_const = self.T - 2 * T_acc

        if self.time < T_acc:
            scale = self.time / T_acc
        elif self.time < T_acc + T_const:
            scale = 1.0
        else:
            scale = (self.T - self.time) / T_acc
        
        vx = scale * v_x
        vy = scale * v_y
        omega = scale * v_omega
        profile = [vx, vy, omega]
        
        return profile
    
    def generate_symmetric_ramp(self, v_x=0.5, v_y=0.0, v_omega=0.0):
        phase_duration = self.T / 4

        if self.time < phase_duration:
            scale = 1 - (self.time / phase_duration)
        elif self.time < 2 * phase_duration:
            scale = -((self.time - phase_duration) / phase_duration)
        elif self.time < 3 * phase_duration:
            scale = -1 + ((self.time - 2 * phase_duration) / phase_duration)
        else:
            scale = (self.time - 3 * phase_duration) / phase_duration

        vx = scale * v_x
        vy = scale * v_y
        omega = scale * v_omega

        return [vx, vy, omega]
    
    def generate_circle_omni(self, radius=1.5, center=(0, 0), clockwise=False):
        omega = -2 * np.pi / self.T if clockwise else 2 * np.pi / self.T
        v = radius * abs(omega)

        angle = omega * self.time
        vx = -v * np.sin(angle)
        vy = v * np.cos(angle)

        return [vx, vy, 0.0]
    
    def generate_circle_forward(self, radius=2.0, linear_speed=0.5, clockwise=False):
        omega = -linear_speed / radius if clockwise else linear_speed / radius

        vx = linear_speed
        vy = 0.0

        return [vx, vy, omega]


if __name__ == "__main__":
    dt = 0.02
    T = 12
    t = np.arange(0, T, dt)

    # Trapezoid
    profile = []
    for i in t:
        cmd = VelocityProfile(T=T, t=i)
        profile.append(cmd.generate_trapezoid())

    profile = np.array(profile)

    plt.plot(t, profile[:, 0], label='vx')
    plt.plot(t, profile[:, 1], label='vy')
    plt.plot(t, profile[:, 2], label='omega')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid(True)
    plt.title("Velocity Command Profile")
    plt.show()

    # Symmetric Ramp
    profile = []
    for i in t:
        cmd = VelocityProfile(T=T, t=i)
        profile.append(cmd.generate_symmetric_ramp())

    profile = np.array(profile)

    plt.plot(t, profile[:, 0], label='vx')
    plt.plot(t, profile[:, 1], label='vy')
    plt.plot(t, profile[:, 2], label='omega')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid(True)
    plt.title("Velocity Command Profile")
    plt.show()

    # Circle (Omnidirectional)
    profile = []
    for i in t:
        cmd = VelocityProfile(T=T, t=i)
        profile.append(cmd.generate_circle_omni(radius=2.0))

    profile = np.array(profile)

    plt.plot(t, profile[:, 0], label='vx (circle)')
    plt.plot(t, profile[:, 1], label='vy (circle)')
    plt.plot(t, profile[:, 2], label='omega (circle)')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid(True)
    plt.title("Circular Path Velocity Commands (Omni)")
    plt.show()

    x, y, theta = 0, 0, 0
    x_path, y_path = [], []
    headings = []
    for vx, vy, w in profile:
        vx_world = vx * np.cos(theta) - vy * np.sin(theta)
        vy_world = vx * np.sin(theta) + vy * np.cos(theta)

        x += vx_world * dt
        y += vy_world * dt
        theta += w * dt

        x_path.append(x)
        y_path.append(y)

        headings.append(theta)

    plt.plot(x_path, y_path)
    plt.axis("equal")
    arrow_every = 10
    for i in range(0, len(x_path), arrow_every):
        dx = 0.2 * np.cos(headings[i])
        dy = 0.2 * np.sin(headings[i])
        plt.arrow(x_path[i], y_path[i], dx, dy,
                head_width=0.05, head_length=0.05, fc='r', ec='r')
    plt.title("Circular Path without Orientation")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True)
    plt.show()

    # Circle (Forward)
    profile = []
    for i in t:
        cmd = VelocityProfile(T=T, t=i)
        profile.append(cmd.generate_circle_forward(radius=2.0, linear_speed=1.0))

    profile = np.array(profile)

    plt.plot(t, profile[:, 0], label='vx (circle)')
    plt.plot(t, profile[:, 1], label='vy (circle)')
    plt.plot(t, profile[:, 2], label='omega (circle)')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.grid(True)
    plt.title("Circular Path Velocity Commands (Forward)")
    plt.show()

    x, y, theta = 0, 0, 0
    x_path, y_path = [], []
    headings = []
    for vx, vy, w in profile:
        vx_world = vx * np.cos(theta) - vy * np.sin(theta)
        vy_world = vx * np.sin(theta) + vy * np.cos(theta)

        x += vx_world * dt
        y += vy_world * dt
        theta += w * dt

        x_path.append(x)
        y_path.append(y)

        headings.append(theta)

    plt.plot(x_path, y_path)
    plt.axis('equal')
    arrow_every = 10
    for i in range(0, len(x_path), arrow_every):
        dx = 0.2 * np.cos(headings[i])
        dy = 0.2 * np.sin(headings[i])
        plt.arrow(x_path[i], y_path[i], dx, dy,
                head_width=0.05, head_length=0.05, fc='r', ec='r')
    plt.title("Circular Path with Orientation")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True)
    plt.show()