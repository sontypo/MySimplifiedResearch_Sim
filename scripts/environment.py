import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Circle, Arrow
import random as rd

# Get Tableau colors from Matplotlib
tableau_colors = mcolors.TABLEAU_COLORS

class Environment:
    def __init__(self, hallway_length=15.0, hallway_width=6.3):
        # Hallway dimensions
        self.hallway_length = hallway_length
        self.hallway_width = hallway_width

        # Initialize humans
        self.human_positions = [np.array([-1.0, 1.5]), np.array([5.0, -1.0])]
        self.human_velocities = [np.array([1.4, rd.random()]), np.array([-1.2, -rd.random()])]
        self.human_radius = 0.4  # Size of humans

        # Create a figure with two main sections
        self.fig = plt.figure(figsize=(16, 8))
        self.ax_env = self.fig.add_subplot(121)  # Left: Robot-environment interaction

        # Initialize robot-environment interaction plot
        self.ax_env.set_xlim(-5.5, 15.5)
        self.ax_env.set_ylim(-4, 4)
        self.ax_env.set_aspect("equal")
        self.ax_env.set_title("Robot-Environment Interaction")
        self.ax_env.set_xlabel("X (m)")
        self.ax_env.set_ylabel("Y (m)")

        # Draw hallway boundaries
        self.ax_env.axhline(y=hallway_width / 2, color="gray", linestyle="-", linewidth=4, label="Left Boundary")
        self.ax_env.axhline(y=-hallway_width / 2, color="gray", linestyle="-", linewidth=4, label="Right Boundary")

        # Draw grid
        self.ax_env.grid(True, linestyle="--", alpha=0.5)

        # Initialize human visuals
        self.human_visuals = [Circle(pos, radius=self.human_radius, facecolor=tableau_colors["tab:blue"], edgecolor='blue', label="Human") for pos in self.human_positions]
        for human_visual in self.human_visuals:
            self.ax_env.add_patch(human_visual)

        # Initialize robot visual
        self.robot_visual = Circle((0, 0), radius=0.45, facecolor=tableau_colors["tab:orange"], edgecolor='red', label="Robot")
        self.force_arrow = Arrow(0, 0, 0, 0, color="green", width=0.1, label="Total Force")
        self.ax_env.add_patch(self.robot_visual)
        self.ax_env.add_patch(self.force_arrow)
        self.ax_env.legend()
        
        # Move legend outside the environment plot
        self.ax_env.legend(loc="upper left", bbox_to_anchor=(1.05, 1))  # Place legend outside

        # Create 4 subplots for force components
        self.ax_forces = self.fig.add_subplot(422)  # Top-right: F_des
        self.ax_forces.set_title("F_des Components")
        self.ax_forces.set_ylabel("Force (N)")
        self.ax_forces.grid(True)

        self.ax_forces2 = self.fig.add_subplot(424)  # Second from top: F_soc
        self.ax_forces2.set_title("F_soc Components")
        self.ax_forces2.set_ylabel("Force (N)")
        self.ax_forces2.grid(True)

        self.ax_forces3 = self.fig.add_subplot(426)  # Third from top: F_bound
        self.ax_forces3.set_title("F_bound Components")
        self.ax_forces3.set_ylabel("Force (N)")
        self.ax_forces3.grid(True)

        self.ax_forces4 = self.fig.add_subplot(428)  # Bottom-right: F_total
        self.ax_forces4.set_title("F_total Components")
        self.ax_forces4.set_xlabel("Time (s)")
        self.ax_forces4.set_ylabel("Force (N)")
        self.ax_forces4.grid(True)

        # Store time and force data
        self.time_data = []
        self.force_data = {
            "F_des": {"x": [], "y": []},
            "F_soc": {"x": [], "y": []},
            "F_bound": {"x": [], "y": []},
            "F_total": {"x": [], "y": []},
        }
        
        # Adjust subplot spacing
        plt.tight_layout()
        plt.subplots_adjust(left=0.1, right=0.7, wspace=0.3)  # Enlarge left subplot

    def update_humans(self, dt):
        """Update human positions based on their velocities."""
        for i in range(len(self.human_positions)):
            self.human_positions[i] += self.human_velocities[i] * dt

    def update_visuals(self, robot_position, force_vector, force_components, time):
        """Update the positions of the robot, humans, and force arrow, and update force plots."""
        # Update robot position
        self.robot_visual.center = robot_position

        # Update force arrow
        self.force_arrow.remove()
        self.force_arrow = Arrow(
            robot_position[0], robot_position[1],
            force_vector[0] * 0.1, force_vector[1] * 0.1,  # Scale force for visualization
            color="green", width=0.1, label="Total Force"
        )
        self.ax_env.add_patch(self.force_arrow)

        # Update human positions
        for i, human_visual in enumerate(self.human_visuals):
            human_visual.center = self.human_positions[i]

        # Update force plots
        self.time_data.append(time)
        for force, components in force_components.items():
            self.force_data[force]["x"].append(components[0])
            self.force_data[force]["y"].append(components[1])

        # Clear and update force subplots
        self.ax_forces.clear()
        self.ax_forces.plot(self.time_data, self.force_data["F_des"]["x"], label="F_des (x)")
        self.ax_forces.plot(self.time_data, self.force_data["F_des"]["y"], label="F_des (y)")
        self.ax_forces.set_title("F_des Components")
        self.ax_forces.set_ylabel("Force (N)")
        self.ax_forces.legend()
        self.ax_forces.grid(True)

        self.ax_forces2.clear()
        self.ax_forces2.plot(self.time_data, self.force_data["F_soc"]["x"], label="F_soc (x)")
        self.ax_forces2.plot(self.time_data, self.force_data["F_soc"]["y"], label="F_soc (y)")
        self.ax_forces2.set_title("F_soc Components")
        self.ax_forces2.set_ylabel("Force (N)")
        self.ax_forces2.legend()
        self.ax_forces2.grid(True)

        self.ax_forces3.clear()
        self.ax_forces3.plot(self.time_data, self.force_data["F_bound"]["x"], label="F_bound (x)")
        self.ax_forces3.plot(self.time_data, self.force_data["F_bound"]["y"], label="F_bound (y)")
        self.ax_forces3.set_title("F_bound Components")
        self.ax_forces3.set_ylabel("Force (N)")
        self.ax_forces3.legend()
        self.ax_forces3.grid(True)

        self.ax_forces4.clear()
        self.ax_forces4.plot(self.time_data, self.force_data["F_total"]["x"], label="F_total (x)")
        self.ax_forces4.plot(self.time_data, self.force_data["F_total"]["y"], label="F_total (y)")
        self.ax_forces4.set_title("F_total Components")
        self.ax_forces4.set_xlabel("Time (s)")
        self.ax_forces4.set_ylabel("Force (N)")
        self.ax_forces4.legend()
        self.ax_forces4.grid(True)

        # Redraw the plot
        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)

    def get_human_positions(self):
        """Return the current positions of humans."""
        return self.human_positions