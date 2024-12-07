
from manimlib import *
from scipy.integrate import solve_ivp
import numpy as np

def scaled_radius(masses):
    max_mass = np.max(masses)
    min_mass = np.min(masses)
    max_radius = 0.5
    min_radius = 0.2
    mass_range = np.log(max_mass) - np.log(min_mass)
    normalized_mass = (np.log(masses) - np.log(min_mass)) / mass_range
    return min_radius + normalized_mass * (max_radius - min_radius)

def spaced_axes(min, max, step):
    if np.abs(max - min) < 1e-2:
        avg = (max + min) / 2
        return (avg - 10, avg + 10, 1)
    return (min, max, step)

class NBodySimulation:
    def __init__(self, masses, initial_positions, initial_velocities, G=6.67430e-11):
        self.masses = np.array(masses)  # Array of masses
        self.initial_positions = np.array(initial_positions)  # Initial positions
        self.initial_velocities = np.array(initial_velocities)  # Initial velocities
        self.G = G  # Gravitational constant
        self.n_bodies = len(masses)  # Number of bodies

    def _compute_accelerations(self, positions):
        accelerations = np.zeros_like(positions)
        for i in range(self.n_bodies):
            for j in range(self.n_bodies):
                # print(i, j)
                if i != j:
                    r_ij = positions[j] - positions[i]
                    distance = np.linalg.norm(r_ij)
                    if distance > 1e-5:  # Avoid division by zero
                        accelerations[i] += self.G * self.masses[j] * r_ij / distance**2
        return accelerations

    def _dynamics(self, t, state):
        positions = state[:3 * self.n_bodies].reshape((self.n_bodies, 3))
        velocities = state[3 * self.n_bodies:].reshape((self.n_bodies, 3))
        accelerations = self._compute_accelerations(positions)
        return np.concatenate([velocities.flatten(), accelerations.flatten()])

    def simulate(self, t_span, time_step):
        # Initial state: Flatten positions and velocities
        initial_state = np.concatenate([self.initial_positions.flatten(), self.initial_velocities.flatten()])

        # Time points
        times = np.arange(t_span[0], t_span[1], time_step)

        # Solve ODE
        solution = solve_ivp(
            self._dynamics,
            t_span=t_span,
            y0=initial_state,
            t_eval=times,
            method='RK45',
            # rtol=1e-8,
            # atol=1e-10
        )

        # Extract positions and velocities
        times = solution.t
        positions = solution.y[:3 * self.n_bodies].reshape((self.n_bodies, 3, len(times)))
        velocities = solution.y[3 * self.n_bodies:].reshape((self.n_bodies, 3, len(times)))

        # Compute energies
        kinetic_energy = np.zeros_like(times)
        potential_energy = np.zeros_like(times)

        for k, time in enumerate(times):
            vel = velocities[:, :, k]
            pos = positions[:, :, k]

            # Kinetic energy: 1/2 * m * v^2
            kinetic_energy[k] = 0.5 * np.sum(self.masses[:, None] * np.sum(vel**2, axis=1))

            # Potential energy: -G * m1 * m2 / r
            for i in range(self.n_bodies):
                for j in range(i + 1, self.n_bodies):
                    r_ij = np.linalg.norm(pos[j] - pos[i])
                    if r_ij > 1e-5:
                        potential_energy[k] -= self.G * self.masses[i] * self.masses[j] / r_ij

        return times, positions, velocities, kinetic_energy, potential_energy


class NBodyGravityScene(InteractiveScene):
    def construct(self):
        masses = [10.0, 20.0, 30.0]
        initial_positions = [[-10.0, 10.0, 1.0], [0.0, 0.0, 2.0], [10.0, 10.0, -1.0]]
        initial_velocities = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        total_time = 1000
        time_step = 0.5
        G = 1.0
        sim_time = 20

        simulation = NBodySimulation(masses, initial_positions, initial_velocities, G=G)
        times, positions, velocities, kinetic_energy, potential_energy = simulation.simulate((0, total_time), time_step)

        min_x = int(np.floor(np.min(positions[:, 0, :])))
        max_x = int(np.ceil(np.max(positions[:, 0, :])))
        step_x = (max_x - min_x) // 20
        min_y = int(np.floor(np.min(positions[:, 1, :])))
        max_y = int(np.ceil(np.max(positions[:, 1, :])))
        step_y = (max_y - min_y) // 20
        min_z = int(np.floor(np.min(positions[:, 2, :])))
        max_z = int(np.ceil(np.max(positions[:, 2, :])))
        step_z = (max_z - min_z) // 20

        min_x, max_x, step_x = spaced_axes(min_x, max_x, step_x)
        min_y, max_y, step_y = spaced_axes(min_y, max_y, step_y)
        min_z, max_z, step_z = spaced_axes(min_z, max_z, step_z)

        axes = ThreeDAxes(
            x_range=(min_x, max_x, step_x),
            y_range=(min_y, max_y, step_y),
            z_range=(min_z, max_z, step_z),
            width=16,
            height=16,
            depth=8,
        )
        axes.set_width(FRAME_WIDTH)
        axes.center()

        self.frame.reorient(43, 76, 1, IN, 10)
        self.frame.add_updater(lambda m, dt: m.increment_theta(dt * 3 * DEGREES))
        self.add(axes)

        n = len(masses)
        colors = color_gradient([BLUE, RED], n)

        curves = VGroup()
        for i, color in enumerate(colors):
            points = positions[i, :, :].reshape((3, -1)).T
            curve = VMobject().set_points_smoothly(axes.c2p(*points.T))
            curve.set_stroke(color, 1, opacity=0.25)
            curves.add(curve)

        curves.set_stroke(width=2, opacity=1)
            

        radii = scaled_radius(masses)

        dots = Group(TrueDot(color=color, radius=0.1) for color, radius in zip(colors, radii))

        def update_dots(dots, curves=curves):
            for dot, curve in zip(dots, curves):
                dot.move_to(curve.get_end())

        dots.add_updater(update_dots)

        tail = VGroup(
            TracingTail(dot, time_traced=20).match_color(dot)
            for dot in dots
        )

        self.add(dots)
        self.add(tail)
        curves.set_opacity(0)
        self.play(
            *(
                ShowCreation(curve, rate_func=linear)
                for curve in curves
            ),
            run_time=20,
        )

        self.wait(1)
